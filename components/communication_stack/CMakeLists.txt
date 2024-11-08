set(pb_csrcs
    "${CMAKE_CURRENT_LIST_DIR}/socket_manager/protobuf_msgs/messages.pb.c")

if(NOT CMAKE_BUILD_EARLY_EXPANSION)
    add_custom_command(
      OUTPUT ${pb_csrcs}
      COMMAND ${COMPONENT_DIR}/../nanopb/generator-bin/protoc -I ${COMPONENT_DIR}/socket_manager --nanopb_out=${CMAKE_CURRENT_LIST_DIR}/socket_manager/protobuf_msgs messages.proto 
      DEPENDS ${COMPONENT_DIR}/socket_manager/messages.proto
      VERBATIM)
endif()

idf_component_register(
    SRCS 
        wifi_manager/wifi_manager.cc
        socket_manager/socket_manager.cc
        socket_manager/common.cc
        ${pb_csrcs}
    INCLUDE_DIRS 
        wifi_manager
        socket_manager
        socket_manager/protobuf_msgs
    REQUIRES
        nanopb
    PRIV_REQUIRES
        driver
        esp_event
        esp_http_server
        esp_netif
        spiffs
        esp_wifi
        nvs_flash
        wifi_provisioning
        hardware_drivers
)
