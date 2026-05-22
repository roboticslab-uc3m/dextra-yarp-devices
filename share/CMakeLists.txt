yarp_configure_external_installation(teo-configuration-files NO_PATH_D)
yarp_configure_external_installation(teo-gz-sim-models NO_PATH_D)
yarp_configure_external_installation(teo-openrave-models NO_PATH_D)

yarp_install(DIRECTORY contexts/
             DESTINATION ${TEO-CONFIGURATION-FILES_CONTEXTS_INSTALL_DIR})

yarp_install(DIRECTORY gz-sim/
             DESTINATION ${TEO-GZ-SIM-MODELS_DATA_INSTALL_DIR})

yarp_install(DIRECTORY openrave/
             DESTINATION ${TEO-OPENRAVE-MODELS_DATA_INSTALL_DIR})

yarp_install(DIRECTORY robots/
             DESTINATION ${TEO-CONFIGURATION-FILES_ROBOTS_INSTALL_DIR})
