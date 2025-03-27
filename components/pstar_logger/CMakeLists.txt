# Initialize lists
# Handler and Storage are always needed now, as handler calls storage functions
# even for buffering/mutex management when SD is disabled.
set(logger_srcs
  "pstar_log_handler.c"
  "pstar_log_storage.c"
)
# Base requirements (needed by handler or storage's minimal implementation)
set(logger_requires driver esp_common freertos nvs_flash esp_timer pstar_managers)
set(logger_priv_requires "")

# Conditionally add dependencies needed *only* for SD storage features
if(CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED)
  # SD storage requires storage_hal (for SD card) and zlib (if compressing)
  list(APPEND logger_requires pstar_storage_hal)
  if(CONFIG_PSTAR_KCONFIG_LOGGING_COMPRESSION_ENABLED)
      list(APPEND logger_priv_requires zlib)
  endif()
endif()

idf_component_register(
  SRCS ${logger_srcs}
  INCLUDE_DIRS "include"
  REQUIRES ${logger_requires}
  PRIV_REQUIRES ${logger_priv_requires}
)
