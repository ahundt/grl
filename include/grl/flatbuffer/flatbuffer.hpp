#ifndef GRL_FLATBUFFER_HPP
#define GRL_FLATBUFFER_HPP

#include <flatbuffers/flatbuffers.h>
#include <flatbuffers/idl.h>



namespace grl {

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

/// @brief Loads a flatbuffers schema file from disk into a flatbuffers parser.
///
/// @param include_paths JSON ONLY. This is the list of paths describing where to find the
///            flatbuffers schema files (.fbs) required to write the json file.
///            In particular, LogKUKAiiwaFusionTrack.fbs is requred. If the empty
///            string or an empty list is specified, the current working directory
///            will be checked for LogKUKAiiwaFusionTrack.fbs and the other files
///            it depends on.
/// @param read_binary_schema JSON only. The LogKUKAiiwaFusionTrack.fbs and other schemea files
///            can be saved in ascii plaintext format (the default) and in binary format.
///            Set this option to true if you expect to load the schema files in binary format.
/// @param write_binary_schema If "binary_stream" is false data is written using ifstream's
///            text mode, otherwise data is written with no transcoding.
bool ParseSchemaFile(
    flatbuffers::Parser& parser,
    std::string fbs_filename,
    std::vector<std::string> includePaths = std::vector<std::string >(),
    bool read_binary_schema=false)
{

    bool loadfbsfile_ok = true;
    std::string schemafile;
    if(includePaths.empty())
    {

        std::string current_working_dir;
        char buff[2048];
        /// Get the current working directory
        getcwd(buff, 2048);
        current_working_dir = std::string(buff);
        std::cout << "The current working dir: " << current_working_dir << std::endl;
        includePaths.push_back(current_working_dir);
    }

    std::string fbs_fullpath;
    // check if the user passed a full path to the fbs file
    if(flatbuffers::FileExists(fbs_filename.c_str())){
        fbs_fullpath = fbs_filename;
    }
    else
    {
        std::cout << "a full path wasn't passed, so check all the include paths" << std::endl;
        for(auto includePath : includePaths)
        {
          std::string fbs_trypath = flatbuffers::ConCatPathFileName(includePath, fbs_filename);
          if(flatbuffers::FileExists(fbs_trypath.c_str()))
          {
            fbs_fullpath = fbs_trypath;
            // we found it! no need to keep looping
            break;
          }
        }
    }

    // if it is empty none of the files actually existed, return false
    if(fbs_fullpath.empty()) return false;
    if(!flatbuffers::LoadFile(fbs_filename.c_str(), read_binary_schema, &schemafile))
    {
        // something is wrong
        return false;
    }

    // parse fbs schema, so we can use it to parse the data after
    // create a list of char* pointers so we can call Parse
    std::vector<const char *> include_directories;
    for(int i = 0; i < includePaths.size(); i++){
      include_directories.push_back(includePaths[i].c_str());
    }
    include_directories.push_back(nullptr);

    return parser.Parse(schemafile.c_str(), &include_directories[0]);
}


///
/// Save data flatbuffers::DetachedBuffer db into a file binary_file_path,
/// returning true if successful, false otherwise. Prefer reading and
/// writing binary files over json files, because they are much more efficient.
///
/// @param db flatbuffers detached buffer object to write to disk
/// @param binary_file_path full file path to write the binary log file.
///            Empty string indicates this file should not be saved.
///            Please prefer saving binary files and use the file extension .flik.
/// @param fbs_file_path JSON Only. The file name of the flatbuffers schema definition file (.fbs).
///            If this is empty the JSON file will not be saved.
/// @param json_file_path full file path to write the json file,
///            Only write this file for debugging purposes, it is very expensive.
///            The default empty string indicates this file should not be saved.
///            Please use the file extension .json if saving this file.
/// @param include_paths JSON ONLY. This is the list of paths describing where to find the
///            flatbuffers schema files (.fbs) required to write the json file.
///            In particular, LogKUKAiiwaFusionTrack.fbs is requred. If the empty
///            string or an empty list is specified, the current working directory
///            will be checked for LogKUKAiiwaFusionTrack.fbs and the other files
///            it depends on.
/// @param read_binary_schema JSON only. The LogKUKAiiwaFusionTrack.fbs and other schemea files
///            can be saved in ascii plaintext format (the default) and in binary format.
///            Set this option to true if you expect to load the schema files in binary format.
/// @param write_binary_schema If "binary_stream" is false data is written using ifstream's
///            text mode, otherwise data is written with no transcoding.
bool SaveFlatBufferFile(
    const uint8_t* buffer,
    std::size_t size,
    std::string binary_file_path,
    std::string fbs_filename = std::string(),
    std::string json_file_path = std::string(),
    std::vector<std::string> includePaths = std::vector<std::string >(),
    bool read_binary_schema=false,
    bool write_binary_stream=true)
{
    bool ok = true;
    /////////////////////////////////////
    /// Saving BINARY version of file ///
    /////////////////////////////////////
    if(!binary_file_path.empty())
    {
      ok = ok && flatbuffers::SaveFile(binary_file_path.c_str(), reinterpret_cast<const char*>(buffer), size, write_binary_stream);
    }

    ////////////////////////////////////////////
    /// Load fbs file and generate json file ///
    ////////////////////////////////////////////

    if(!json_file_path.empty() && !fbs_filename.empty())
    {
      flatbuffers::Parser parser;

      ok = ok && ParseSchemaFile(parser, fbs_filename, includePaths, read_binary_schema);
      std::string jsongen;
      // now generate text from the flatbuffer binary

      ok = ok && GenerateText(parser, buffer, &jsongen);
      // Write the data get from flatbuffer binary to json file on disk.

      std::ofstream out(json_file_path);
      out << jsongen.c_str();
      out.close();
    }
    return ok;
}

} // namespace grl
#endif // GRL_FLATBUFFER_HPP