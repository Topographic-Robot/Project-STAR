/* components/pstar_bus/include/bus_macros.h */

#ifndef PSTAR_BUS_MACROS_H
#define PSTAR_BUS_MACROS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Macros *********************************************************************/

/**
 * @brief Validates a bus manager and logs errors if invalid.
 * 
 * @param[in] manager Pointer to the bus manager to validate
 * @param[in] tag Tag for error messages
 * @return true if manager is valid, false otherwise
 */
#define PSTAR_BUS_VALIDATE_MANAGER(manager, tag)                           \
  do {                                                                     \
    if ((manager) == NULL) {                                               \
      log_error((tag), "Validation Error", "Bus manager pointer is NULL"); \
      return false;                                                        \
    }                                                                      \
  } while (0)

/**
 * @brief Validates a bus name and logs errors if invalid.
 * 
 * @param[in] name Bus name to validate
 * @param[in] tag Tag for error messages
 * @return true if name is valid, false otherwise
 */
#define PSTAR_BUS_VALIDATE_NAME(name, tag)                      \
  do {                                                          \
    if ((name) == NULL) {                                       \
      log_error((tag), "Validation Error", "Bus name is NULL"); \
      return false;                                             \
    }                                                           \
  } while (0)

/**
 * @brief Validates both a bus manager and name.
 * 
 * @param[in] manager Pointer to the bus manager to validate
 * @param[in] name Bus name to validate
 * @param[in] tag Tag for error messages
 * @return true if both manager and name are valid, false otherwise
 */
#define PSTAR_BUS_VALIDATE_MANAGER_AND_NAME(manager, name, tag) \
  do {                                                          \
    PSTAR_BUS_VALIDATE_MANAGER((manager), (tag));               \
    PSTAR_BUS_VALIDATE_NAME((name), (tag));                     \
  } while (0)

#ifdef __cplusplus
}
#endif

#endif /* PSTAR_BUS_MACROS_H */
