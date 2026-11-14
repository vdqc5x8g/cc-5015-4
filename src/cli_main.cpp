#include "http_server.h"
#include "terrain/export.h"
#include "terrain/json_io.h"
#include "terrain/pipeline.h"
#include "terrain/tolerance.h"
#include "terrain/validate.h"

#include "json.hpp"

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

namespace {

void usage()
{
    std::cout <<
        "usage: terrain <command> [args]\n"
        "  validate <input.json> [--snap-m M] [--znear M]\n"
        "  build    <input.json> [--out-prefix P] [--snap-m M] [--znear M]\n"
        "  export   <result.json> <prefix>   (mesh JSON produced by build --json)\n"
        "  serve    [port] [webroot]\n";
}

std::string readFile(const std::string& path)
{
    std::ifstream f(path, std::ios::binary);
    std::ostringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

} // namespace

int main(int argc, char** argv)
{
    if (argc < 2) {
        usage();
        return 2;
    }
    const std::string command = argv[1];

    if (command == "validate" || command == "build") {
        if (argc < 3) {
            usage();
            return 2;
        }
        const std::string inputPath = argv[2];
        std::string prefix;
        terrain::PipelineOptions options;
        for (int i = 3; i + 1 < argc; ++i) {
            const std::string flag = argv[i];
            if (flag == "--out-prefix")
                prefix = argv[++i];
            else if (flag == "--snap-m")
                options.snapToleranceMeters = std::stod(argv[++i]);
            else if (flag == "--znear")
                options.zNearToleranceMeters = std::stod(argv[++i]);
        }
        const std::string text = readFile(inputPath);
        if (text.empty()) {
            std::cerr << "cannot read " << inputPath << "\n";
            return 1;
        }
        terrain::PipelineResult result = terrain::runPipeline(text, options);
        if (!result.error.empty())
            std::cerr << "error: " << result.error << "\n";

        if (command == "validate") {
            std::cout << terrain::validationReportText(result.validation);
            std::cout << "\nTolerance derived from data:\n"
                      << terrain::describeTolerance(result.tolerance);
            std::cout << "\nRepair candidates (not applied unless selected):\n";
            for (const terrain::RepairCandidate& c : result.validation.candidates)
                std::cout << "  " << c.id << "  " << c.code << "  " << c.label << "\n";
            return result.validation.hasBlockingErrors() ? 1 : 0;
        }

        // build: write the mesh in several formats plus the pipeline JSON.
        if (prefix.empty())
            prefix = "mesh";
        if (!result.mesh.ok) {
            std::cerr << "mesh build failed: " << result.mesh.error << "\n";
            for (const terrain::ValidationIssue& n : result.mesh.notes)
                std::cerr << "  [" << n.severity << "] " << n.message << "\n";
            return 1;
        }
        {
            std::ofstream obj(prefix + ".obj", std::ios::binary);
            obj << terrain::meshToObj(result.mesh, result.tolerance);
            std::ofstream stl(prefix + ".stl", std::ios::binary);
            const std::string bytes = terrain::meshToStlBinary(result.mesh, result.tolerance);
            stl.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
            std::ofstream vcsv(prefix + "_vertices.csv", std::ios::binary);
            vcsv << terrain::meshToCsvVertices(result.mesh);
            std::ofstream tcsv(prefix + "_triangles.csv", std::ios::binary);
            tcsv << terrain::meshToCsvTriangles(result.mesh);
            std::ofstream json(prefix + "_mesh.json", std::ios::binary);
            json << terrain::meshToJson(result.mesh, result.tolerance, result.repaired);
            std::ofstream pipeline(prefix + "_pipeline.json", std::ios::binary);
            pipeline << terrain::pipelineToJson(result, true);
        }
        std::cout << "mesh: " << result.mesh.triangles.size() << " faces, "
                  << result.mesh.vertices.size() << " vertices\n";
        std::cout << "constrained edges: " << result.mesh.edges.size() << "\n";
        std::cout << "triangulation " << result.mesh.triangulationMs << " ms, elevation "
                  << result.mesh.elevationMs << " ms, total " << result.mesh.totalMs << " ms\n";
        std::cout << "elevation sources: observed " << result.mesh.interpolation.observedVertices
                  << ", along-constraint " << result.mesh.interpolation.edgeInterpolatedVertices
                  << ", harmonic " << result.mesh.interpolation.faceInterpolatedVertices
                  << ", boundary " << result.mesh.interpolation.boundaryInterpolatedVertices
                  << ", unobserved " << result.mesh.interpolation.unobservedVertices << "\n";
        std::cout << "coverage: " << result.mesh.interpolation.coverageNote << "\n";
        std::cout << "wrote " << prefix << ".obj/.stl/_vertices.csv/_triangles.csv/_mesh.json/_pipeline.json\n";
        return 0;
    }

    if (command == "export") {
        if (argc < 4) {
            usage();
            return 2;
        }
        // Convert a saved pipeline JSON into OBJ/STL without recomputing.
        // Rebuild from the original text so the same code path is exercised.
        const std::string pipelineText = readFile(argv[2]);
        try {
            nlohmann::json doc = nlohmann::json::parse(pipelineText);
            if (!doc.contains("original_text")) {
                std::cerr << "not a pipeline JSON (missing original_text)\n";
                return 1;
            }
            terrain::PipelineOptions options;
            terrain::PipelineResult result = terrain::runPipeline(doc["original_text"].get<std::string>(), options);
            if (!result.mesh.ok) {
                std::cerr << "rebuild failed: " << result.mesh.error << "\n";
                return 1;
            }
            const std::string prefix = argv[3];
            std::ofstream obj(prefix + ".obj", std::ios::binary);
            obj << terrain::meshToObj(result.mesh, result.tolerance);
            std::ofstream stl(prefix + ".stl", std::ios::binary);
            const std::string bytes = terrain::meshToStlBinary(result.mesh, result.tolerance);
            stl.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
            std::cout << "wrote " << prefix << ".obj/.stl\n";
            return 0;
        } catch (const std::exception& e) {
            std::cerr << "export failed: " << e.what() << "\n";
            return 1;
        }
    }

    if (command == "serve") {
        terrain::ServeOptions options;
        if (argc >= 3)
            options.port = std::stoi(argv[2]);
        if (argc >= 4)
            options.webRoot = argv[3];
        return terrain::serve(options);
    }

    std::cerr << "unknown command\n";
    usage();
    return 2;
}
