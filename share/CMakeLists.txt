yarp_configure_external_installation(teo-configuration-files)

yarp_install(DIRECTORY contexts/launchCanBus
             DESTINATION ${TEO-CONFIGURATION-FILES_CONTEXTS_INSTALL_DIR})

yarp_install(DIRECTORY robots/teo
             DESTINATION ${TEO-CONFIGURATION-FILES_ROBOTS_INSTALL_DIR})
