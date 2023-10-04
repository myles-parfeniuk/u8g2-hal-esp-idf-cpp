file(GLOB_RECURSE SOURCES src/*.cpp)
idf_component_register(SRC_DIRS "src"
                       INCLUDE_DIRS "include"
                       REQUIRES driver freertos u8g2)