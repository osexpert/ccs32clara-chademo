#pragma once
/* definitions for logging */

#define MODULE_LIST(X)                  \
    X(MOD_CONNMGR,    "connmgr",    0)  \
    X(MOD_HOMEPLUG,   "pevslac",   1)  \
    X(MOD_PEV,        "ccs",        2)  \
    X(MOD_QCA,        "qca",        3)  \
    X(MOD_TCP,        "tcp",        4)  \
    X(MOD_TCPTRAFFIC, "tcptraffic", 5)  \
    X(MOD_IPV6,       "ipv6",       6)  \
    X(MOD_SDP,        "sdp",        7)  \
    X(MOD_ETHTRAFFIC, "ethtraffic", 8)  \
    X(MOD_CHA,        "cha",        9)

enum Module : uint32_t
{
#define X(name, text, bit) name = 1u << bit,
    MODULE_LIST(X)
#undef X
};

constexpr const char* ModuleNames[] =
{
#define X(name, text, bit) "[" text "]",
    MODULE_LIST(X)
#undef X
};

inline const char* ModuleName(Module module)
{
    uint32_t v = static_cast<uint32_t>(module);
    if (v == 0)
        return "[unknown]";

    return ModuleNames[__builtin_ctz(v)];
}

/* Here we define, which logging data is produced after power-on */

#define DEFAULT_LOGGINGMASK (MOD_HOMEPLUG | MOD_PEV | MOD_SDP | MOD_TCP | MOD_CONNMGR | MOD_CHA)

extern void log(enum Module module, const char* format, ...);
extern void log_bytes(enum Module module, const char* s, uint8_t* data, uint16_t len);