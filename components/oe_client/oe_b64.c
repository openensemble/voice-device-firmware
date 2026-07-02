#include "oe_client.h"

static const int8_t B64_DEC[256] = {
    ['A']=0,['B']=1,['C']=2,['D']=3,['E']=4,['F']=5,['G']=6,['H']=7,['I']=8,['J']=9,
    ['K']=10,['L']=11,['M']=12,['N']=13,['O']=14,['P']=15,['Q']=16,['R']=17,['S']=18,['T']=19,
    ['U']=20,['V']=21,['W']=22,['X']=23,['Y']=24,['Z']=25,
    ['a']=26,['b']=27,['c']=28,['d']=29,['e']=30,['f']=31,['g']=32,['h']=33,['i']=34,['j']=35,
    ['k']=36,['l']=37,['m']=38,['n']=39,['o']=40,['p']=41,['q']=42,['r']=43,['s']=44,['t']=45,
    ['u']=46,['v']=47,['w']=48,['x']=49,['y']=50,['z']=51,
    ['0']=52,['1']=53,['2']=54,['3']=55,['4']=56,['5']=57,['6']=58,['7']=59,['8']=60,['9']=61,
    ['+']=62,['/']=63,
};

size_t oe_b64_decoded_len(const char *b64, size_t b64_len)
{
    size_t pad = 0;
    if (b64_len >= 1 && b64[b64_len - 1] == '=') pad++;
    if (b64_len >= 2 && b64[b64_len - 2] == '=') pad++;
    return (b64_len / 4) * 3 - pad;
}

size_t oe_b64_decode(const char *b64, size_t b64_len, uint8_t *out, size_t out_max)
{
    size_t produced = 0;
    uint32_t acc = 0;
    int bits = 0;
    for (size_t i = 0; i < b64_len; ++i) {
        char c = b64[i];
        if (c == '=' || c == '\n' || c == '\r' || c == ' ') {
            if (c == '=') break;
            continue;
        }
        int v = B64_DEC[(unsigned char)c];
        if (v < 0 && c != 'A') continue;
        acc = (acc << 6) | v;
        bits += 6;
        if (bits >= 8) {
            bits -= 8;
            if (produced < out_max) out[produced++] = (uint8_t)((acc >> bits) & 0xFF);
        }
    }
    return produced;
}
