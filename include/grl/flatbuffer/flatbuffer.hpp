#ifndef DISTRIBUTED_FLATBUFFER_HPP
#define DISTRIBUTED_FLATBUFFER_HPP

#include <flatbuffers/flatbuffers.h>
#include <flatbuffers/idl.h>

// loads a json flatbuffer from a file
bool LoadJSONFlatbuffer(flatbuffers::Parser& parser, std::string schemaPath, std::string jsonPath, std::string include_directory, bool binary = false)
{
    // source: https://github.com/google/flatbuffers/blob/master/samples/sample_text.cpp

    // load FlatBuffer schema (.fbs) and JSON from disk
    std::string schemafile;
    std::string jsonfile;
    bool ok = flatbuffers::LoadFile(schemaPath.c_str(), binary, &schemafile) &&
            flatbuffers::LoadFile(jsonPath.c_str(), binary, &jsonfile);
    if (!ok) {
    printf("couldn't load files!\n");
    return nullptr;
    }

    // parse schema first, so we can use it to parse the data after
    const char *include_directories[] = { include_directory.c_str(), "flatbuffer", nullptr };
    ok = parser.Parse(schemafile.c_str(), include_directories) &&
        parser.Parse(jsonfile.c_str(), include_directories);
    assert(ok);
    return ok;
}

#endif // DISTRIBUTED_FLATBUFFER_HPP