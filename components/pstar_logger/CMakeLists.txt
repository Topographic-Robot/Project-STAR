# Initialize lists
set(logger_srcs "pstar_log_handler.c") # Handler is always needed
set(logger_requires driver esp_common freertos nvs_flash esp_timer pstar_managers) # Base requirements
set(logger_priv_requires "")

# Conditionally add SD storage source and dependencies
if(CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED)
  list(APPEND logger_srcs "pstar_log_storage.c")
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
