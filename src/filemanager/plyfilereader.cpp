#include "plyfilereader.h"

using namespace utils;

std::shared_ptr<PlyFileReader> PlyFileReader::m_instance = nullptr;

std::shared_ptr<PlyFileReader> PlyFileReader::instance()
{
    if(!m_instance)
    {
        m_instance = std::shared_ptr<PlyFileReader>(new PlyFileReader);
    }
    return m_instance;
}