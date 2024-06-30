({
    getBaseDef: function (fields) {
        let gearing = fields.gearing.value;
        if (fields.outputRadiusMeters) {
            gearing = gearing / fields.outputRadiusMeters.value;
        }
        return `
        name "${["DBSim_Motor", fields.controllerType.value, fields.port.value, gearing, fields.inverted.value, fields.nominalVoltageVolts.value, fields.stallTorqueNewtonMeters.value, fields.stallCurrentAmps.value, fields.freeCurrentAmps.value, fields.freeSpeedRPM.value].join('_')}"
        maxVelocity ${(2 * Math.PI / 60) * fields.freeSpeedRPM.value / gearing}
        ${fields.outputRadiusMeters ? "maxForce" : "maxTorque"} ${fields.stallTorqueNewtonMeters.value * gearing}
        # The documentation about the multiplier field is unclear as to how it applies differently to differnt fields / functions.
        # I think it's best just to implement it ourselves for now.
        # multiplier ${1 / gearing}
        ${(fields.sound.value !== "default") ? "soundUrl IS sound" : ""}
`;
    },
    getDef: function (motorType) {
        const specs = {
            NEO: {
                port: "id",
                nominalVoltageVolts: 12,
                stallTorqueNewtonMeters: 2.6,
                stallCurrentAmps: 105,
                freeCurrentAmps: 1.8,
                freeSpeedRPM: 5676
            },
            MiniCIM: {
                port: "port",
                nominalVoltageVolts: 12,
                stallTorqueNewtonMeters: 1.41,
                stallCurrentAmps: 89,
                freeCurrentAmps: 3,
                freeSpeedRPM: 5840
            },
        };
        let spec = specs[motorType];
        return `
        controllerType IS controllerType
        port IS ${spec.port}
        gearing IS gearing
        inverted IS inverted
        nominalVoltageVolts ${spec.nominalVoltageVolts}
        stallTorqueNewtonMeters ${spec.stallTorqueNewtonMeters}
        stallCurrentAmps ${spec.stallCurrentAmps}
        freeCurrentAmps ${spec.freeCurrentAmps}
        freeSpeedRPM ${spec.freeSpeedRPM}
        sound IS sound
    `;
    },
})
