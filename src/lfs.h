#pragma once

#include <LittleFS.h>

struct littlefs
{
    littlefs()
    {
        // Keep LittleFS mounted for the whole runtime to avoid repeated
        // mount/unmount stalls that can impact Wi‑Fi responsiveness.
        LittleFS.begin(true);
    }

    ~littlefs()
    {
        // Do not call LittleFS.end() on scope exit; keep FS mounted.
    }

    template<typename T>
    T load(const char* fname)
    {
        if( auto f = LittleFS.open(fname, "r") )
        {
            T data;

            if( f.readBytes(reinterpret_cast<char*>(&data), sizeof(T)) == sizeof(T) )
            {
                return data;
            }
        }

        return {};
    }

    template<typename T>
    void save(const char* fname, const T& data)
    {
        if( auto f = LittleFS.open(fname, "w") )
        {
            f.write(reinterpret_cast<const uint8_t*>(&data), sizeof(T));
        }
    }
};

