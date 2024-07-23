({
    assertValidPoweredJointDevices: function (devices, fieldDesc, assert) {
        let hasBrake = false;
        let motorDevice = null;
        let encoderDevice = null;
        for (let i = 0; i < devices.length; i++) {
            let node_name = devices[i].node_name;
            if (node_name.endsWith("Brake")) {
                hasBrake = true;
            } else if (node_name.endsWith("Motor")) {
                motorDevice = devices[i];
            } else if (node_name.endsWith("Encoder")) {
                encoderDevice = devices[i];
            }
        }
        assert(hasBrake, fieldDesc + " is missing the required DBSBrake device!");
        assert(motorDevice !== null, fieldDesc + " is missing the required motor device!");
        let motorType = motorDevice?.node_name;
        let controllerType = motorDevice?.fields.controllerType.value;
        if (controllerType?.startsWith("Spark")) {
            assert(encoderDevice !== null, fieldDesc + " is using a " + controllerType + " motor controller but is missing the required encoder device! "
                + "Consider adding a REVBuiltinEncoder.");
        } else if (controllerType === "PWM") {
            let encoderType = encoderDevice?.node_name;
            assert(!encoderType?.endsWith("BuiltinEncoder"), fieldDesc + " is using a " + controllerType + " motor controller, so a built-in encoder is not allowed!");
        }
    },
})
