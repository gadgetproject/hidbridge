cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(hidbridge)

file(GLOB soft_SRCS "*.c")
file(GLOB soft_HDRS "*.h")
target_sources(app PRIVATE ${soft_SRCS} INTERFACE ${soft_HDRS})
