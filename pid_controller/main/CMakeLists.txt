idf_component_register(
    SRCS 
        "main.cpp"
        "PIDController.cpp"
        "MAX6675Sensor.cpp"
        "HeaterController.cpp"
        "ZeroCrossDetector.cpp"
        "SetpointController.cpp"
        "OLEDDisplay.cpp"
    INCLUDE_DIRS "."
)
