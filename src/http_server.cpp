#include "http_server.h"

#include "terrain/export.h"
#include "terrain/pipeline.h"

#include "json.hpp"

#include <algorithm>
#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <dirent.h>
#include <fstream>
#include <iostream>
#include <netinet/in.h>
#include <sstream>
#include <string>
#include <sys/socket.h>
#include <unistd.h>
#include <vector>

namespace terrain {

namespace {

std::string readFile(const std::string& path)
{
    std::ifstream f(path, std::ios::binary);
    std::ostringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

std::string mimeFor(const std::string& path)
{
    const std::size_t dot = path.find_last_of('.');
    const std::string ext = dot == std::string::npos ? "" : path.substr(dot);
    if (ext == ".html")
        return "text/html; charset=utf-8";
    if (ext == ".js")
        return "text/javascript; charset=utf-8";
    if (ext == ".css")
        return "text/css; charset=utf-8";
    if (ext == ".json")
        return "application/json; charset=utf-8";
    if (ext == ".png")
        return "image/png";
    if (ext == ".svg")
        return "image/svg+xml";
    return "application/octet-stream";
}

std::string urlDecode(const std::string& in)
{
    std::string out;
    out.reserve(in.size());
    for (std::size_t i = 0; i < in.size(); ++i) {
        if (in[i] == '%' && i + 2 < in.size()) {
            const std::string hex = in.substr(i + 1, 2);
            char* end = nullptr;
            const long v = std::strtol(hex.c_str(), &end, 16);
            if (end && *end == '\0') {
                out.push_back(static_cast<char>(v));
                i += 2;
                continue;
            }
        }
        if (in[i] == '+')
            out.push_back(' ');
        else
            out.push_back(in[i]);
    }
    return out;
}

// Reject traversal and absolute paths; only a plain file name is allowed.
bool safeName(const std::string& name)
{
    if (name.empty() || name == "." || name == "..")
        return false;
    for (char c : name) {
        if (c == '/' || c == '\\' || c == '\0')
            return false;
    }
    return true;
}

void sendAll(int fd, const std::string& data)
{
    std::size_t off = 0;
    while (off < data.size()) {
        const ssize_t n = ::send(fd, data.data() + off, data.size() - off, MSG_NOSIGNAL);
        if (n <= 0) {
            if (errno == EINTR)
                continue;
            break;
        }
        off += static_cast<std::size_t>(n);
    }
}

void respond(int fd, int status, const std::string& contentType, const std::string& body)
{
    const char* reason = "OK";
    if (status == 400)
        reason = "Bad Request";
    else if (status == 404)
        reason = "Not Found";
    else if (status == 405)
        reason = "Method Not Allowed";
    else if (status == 500)
        reason = "Internal Server Error";

    std::ostringstream head;
    head << "HTTP/1.1 " << status << ' ' << reason << "\r\n"
         << "Content-Type: " << contentType << "\r\n"
         << "Content-Length: " << body.size() << "\r\n"
         << "Cache-Control: no-store\r\n"
         << "Access-Control-Allow-Origin: *\r\n"
         << "Connection: close\r\n\r\n";
    sendAll(fd, head.str() + body);
}

std::string jsonError(const std::string& message)
{
    nlohmann::json j;
    j["ok"] = false;
    j["error"] = message;
    return j.dump();
}

bool readRequest(int fd, std::string& method, std::string& path, std::string& body)
{
    std::string buffer;
    char chunk[8192];
    std::size_t headerEnd = std::string::npos;
    while (true) {
        const ssize_t n = ::recv(fd, chunk, sizeof(chunk), 0);
        if (n <= 0)
            return false;
        buffer.append(chunk, static_cast<std::size_t>(n));
        headerEnd = buffer.find("\r\n\r\n");
        if (headerEnd != std::string::npos || buffer.size() > (1u << 20))
            break;
    }
    if (headerEnd == std::string::npos)
        return false;

    const std::string head = buffer.substr(0, headerEnd);
    body = buffer.substr(headerEnd + 4);

    std::istringstream hs(head);
    std::string requestLine;
    std::getline(hs, requestLine);
    std::istringstream rl(requestLine);
    std::string version;
    if (!(rl >> method >> path >> version))
        return false;

    std::size_t contentLength = 0;
    std::string line;
    while (std::getline(hs, line)) {
        if (!line.empty() && line.back() == '\r')
            line.pop_back();
        const std::string key = "Content-Length:";
        if (line.size() >= key.size() && line.compare(0, key.size(), key) == 0) {
            contentLength = static_cast<std::size_t>(std::stoul(line.substr(key.size())));
        }
    }
    while (body.size() < contentLength) {
        const ssize_t n = ::recv(fd, chunk, sizeof(chunk), 0);
        if (n <= 0)
            break;
        body.append(chunk, static_cast<std::size_t>(n));
    }
    return true;
}

std::string handleApiRun(const std::string& body, std::string& error)
{
    try {
        const nlohmann::json req = nlohmann::json::parse(body);
        if (!req.contains("input_text") || !req["input_text"].is_string()) {
            error = "request must contain input_text (the JSON document as a string)";
            return "";
        }
        PipelineOptions options;
        if (req.contains("repairs") && req["repairs"].is_array()) {
            for (const auto& id : req["repairs"]) {
                if (id.is_string())
                    options.selectedRepairIds.push_back(id.get<std::string>());
            }
        }
        if (req.contains("snap_m") && req["snap_m"].is_number())
            options.snapToleranceMeters = req["snap_m"].get<double>();
        if (req.contains("znear_m") && req["znear_m"].is_number())
            options.zNearToleranceMeters = req["znear_m"].get<double>();

        const PipelineResult result = runPipeline(req["input_text"].get<std::string>(), options);
        return pipelineToJson(result, true);
    } catch (const std::exception& e) {
        error = std::string("bad request: ") + e.what();
        return "";
    }
}

std::string handleApiExport(const std::string& body, std::string& error)
{
    try {
        const nlohmann::json req = nlohmann::json::parse(body);
        if (!req.contains("input_text") || !req["input_text"].is_string()) {
            error = "request must contain input_text";
            return "";
        }
        PipelineOptions options;
        if (req.contains("repairs") && req["repairs"].is_array()) {
            for (const auto& id : req["repairs"]) {
                if (id.is_string())
                    options.selectedRepairIds.push_back(id.get<std::string>());
            }
        }
        const PipelineResult result = runPipeline(req["input_text"].get<std::string>(), options);
        if (!result.mesh.ok) {
            error = result.mesh.error.empty() ? "mesh build failed" : result.mesh.error;
            return "";
        }
        const std::string format = req.value("format", std::string("obj"));
        if (format == "stl")
            return meshToStlBinary(result.mesh, result.tolerance);
        if (format == "csv")
            return meshToCsvVertices(result.mesh) + meshToCsvTriangles(result.mesh);
        if (format == "json")
            return meshToJson(result.mesh, result.tolerance, result.repaired);
        if (format == "pipeline")
            return pipelineToJson(result, true);
        return meshToObj(result.mesh, result.tolerance);
    } catch (const std::exception& e) {
        error = std::string("export request failed: ") + e.what();
        return "";
    }
}

void handleClient(int fd, const ServeOptions& options)
{
    std::string method, path, body;
    if (!readRequest(fd, method, path, body)) {
        respond(fd, 400, "application/json", jsonError("malformed request"));
        return;
    }
    const std::size_t q = path.find('?');
    if (q != std::string::npos)
        path = path.substr(0, q);

    if (path == "/api/run") {
        if (method != "POST") {
            respond(fd, 405, "application/json", jsonError("use POST"));
            return;
        }
        std::string error;
        const std::string json = handleApiRun(body, error);
        if (!error.empty()) {
            respond(fd, 400, "application/json", jsonError(error));
            return;
        }
        respond(fd, 200, "application/json; charset=utf-8", json);
        return;
    }

    // Re-run the current document with the chosen repairs and return an export
    // file, so the browser can download the real mesh without a local disk.
    if (path == "/api/export") {
        if (method != "POST") {
            respond(fd, 405, "application/json", jsonError("use POST"));
            return;
        }
        std::string error;
        const std::string file = handleApiExport(body, error);
        if (!error.empty()) {
            respond(fd, 400, "application/json", jsonError(error));
            return;
        }
        respond(fd, 200, "application/octet-stream", file);
        return;
    }

    if (method != "GET" && method != "HEAD") {
        respond(fd, 405, "application/json", jsonError("use GET"));
        return;
    }

    if (path == "/api/samples") {
        nlohmann::json list = nlohmann::json::array();
        if (DIR* dir = opendir(options.samplesDir.c_str())) {
            std::vector<std::string> names;
            while (dirent* entry = readdir(dir)) {
                const std::string name = entry->d_name;
                if (name.size() > 5 && name.compare(name.size() - 5, 5, ".json") == 0)
                    names.push_back(name);
            }
            closedir(dir);
            std::sort(names.begin(), names.end());
            for (const std::string& name : names)
                list.push_back(name);
        }
        respond(fd, 200, "application/json; charset=utf-8", list.dump());
        return;
    }

    if (path.rfind("/api/sample/", 0) == 0) {
        const std::string name = urlDecode(path.substr(std::string("/api/sample/").size()));
        if (!safeName(name)) {
            respond(fd, 400, "application/json", jsonError("invalid sample name"));
            return;
        }
        const std::string text = readFile(options.samplesDir + "/" + name);
        if (text.empty()) {
            respond(fd, 404, "application/json", jsonError("sample not found"));
            return;
        }
        respond(fd, 200, "application/json; charset=utf-8", text);
        return;
    }

    // Static: /vendor/* from bundled Three.js, everything else from web root.
    std::string filePath;
    if (path.rfind("/vendor/", 0) == 0) {
        const std::string name = path.substr(std::string("/vendor/").size());
        if (!safeName(name)) {
            respond(fd, 400, "text/plain; charset=utf-8", "invalid path");
            return;
        }
        filePath = options.thirdPartyDir + "/" + name;
    } else {
        const std::string name = path == "/" ? "index.html" : path.substr(1);
        if (!safeName(name)) {
            respond(fd, 400, "text/plain; charset=utf-8", "invalid path");
            return;
        }
        filePath = options.webRoot + "/" + name;
    }

    std::string content = readFile(filePath);
    if (content.empty()) {
        respond(fd, 404, "text/plain; charset=utf-8", "not found: " + path);
        return;
    }
    if (method == "HEAD")
        content.clear();
    respond(fd, 200, mimeFor(filePath), content);
}

} // namespace

int serve(const ServeOptions& options)
{
    const int server = ::socket(AF_INET, SOCK_STREAM, 0);
    if (server < 0) {
        std::cerr << "socket: " << std::strerror(errno) << "\n";
        return 1;
    }
    int yes = 1;
    ::setsockopt(server, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = htons(static_cast<uint16_t>(options.port));
    if (::bind(server, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        std::cerr << "bind port " << options.port << ": " << std::strerror(errno) << "\n";
        ::close(server);
        return 1;
    }
    if (::listen(server, 16) < 0) {
        std::cerr << "listen: " << std::strerror(errno) << "\n";
        ::close(server);
        return 1;
    }

    std::cout << "terrain viewer: http://127.0.0.1:" << options.port << "/\n";
    std::cout << "  web root:  " << options.webRoot << "\n";
    std::cout << "  samples:   " << options.samplesDir << "\n";
    std::cout.flush();

    while (true) {
        sockaddr_in peer{};
        socklen_t peerLen = sizeof(peer);
        const int client = ::accept(server, reinterpret_cast<sockaddr*>(&peer), &peerLen);
        if (client < 0) {
            if (errno == EINTR)
                continue;
            break;
        }
        handleClient(client, options);
        ::close(client);
    }
    ::close(server);
    return 0;
}

} // namespace terrain
