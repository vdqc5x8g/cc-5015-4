#pragma once

#include <string>

namespace terrain {

struct ServeOptions {
    int port = 8787;
    std::string webRoot = "web";
    std::string samplesDir = "tests/data";
    std::string assetsDir = "assets";
    std::string thirdPartyDir = "third_party/web";
};

// Blocking local HTTP server. Routes:
//   GET  /                      static UI
//   GET  /vendor/<file>         bundled Three.js
//   GET  /api/samples           list sample JSON files
//   GET  /api/sample/<name>     raw sample JSON text
//   GET  /api/assets            terrain asset index (recipes + derived files)
//   GET  /api/asset/<id>        {asset_text, index_entry, verification}
//   POST /api/run               body {input_text, repairs[], snap_m, znear_m}
//                               -> pipeline JSON with mesh
int serve(const ServeOptions& options);

} // namespace terrain
