// Copyright 2015-2024 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#include <Log.h>
#include <cstdarg>
#include <cstdio>

void log_(int level, const char* fmt, ...)
{

  if (level < mobilinkd::tnc::log().level_) return;
  va_list args;
  va_start(args, fmt);
  vprintf(fmt, args);
  va_end(args);
  printf("\r\n");
}

namespace mobilinkd { namespace tnc {

#ifdef KISS_LOGGING

Log& log(void) {
    static Log log(Log::Level::debug);
    return log;
}

#endif

void Log::log(Level level, const char* fmt, ...) {

    if (level < level_) return;
    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
    printf("\r\n");
}

}} // mobilinkd::tnc

