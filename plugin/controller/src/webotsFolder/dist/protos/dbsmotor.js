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
            # The documentation about the multiplier field is unclear as to how it applies differently to different fields / functions.
            # I think it's best just to implement it ourselves for now.
            # multiplier ${1 / gearing}
            ${(fields.sound.value !== "default") ? "soundUrl IS sound" : ""}
        `;
    },
    getDef: function (motorType) {
        const specs = {
            AndyMark9015: {
                nominalVoltageVolts: 12,
                stallTorqueNewtonMeters: 0.36,
                stallCurrentAmps: 71,
                freeCurrentAmps: 3.7,
                freeSpeedRPM: 14270
            },
            AndyMarkRs775_125: {
                nominalVoltageVolts: 12,
                stallTorqueNewtonMeters: 0.28,
                stallCurrentAmps: 18,
                freeCurrentAmps: 1.6,
                freeSpeedRPM: 5800
            },
            Bag: {
                nominalVoltageVolts: 12,
                stallTorqueNewtonMeters: 0.43,
                stallCurrentAmps: 53,
                freeCurrentAmps: 1.8,
                freeSpeedRPM: 13180
            },
            BanebotsRs550: {
                port: "port",
                nominalVoltageVolts: 12,
                stallTorqueNewtonMeters: 0.38,
                stallCurrentAmps: 84,
                freeCurrentAmps: 0.4,
                freeSpeedRPM: 19000
            },
            BanebotsRs775: {
                nominalVoltageVolts: 12,
                stallTorqueNewtonMeters: 0.72,
                stallCurrentAmps: 97,
                freeCurrentAmps: 2.7,
                freeSpeedRPM: 13050
            },
            CIM: {
                nominalVoltageVolts: 12,
                stallTorqueNewtonMeters: 2.42,
                stallCurrentAmps: 133,
                freeCurrentAmps: 2.7,
                freeSpeedRPM: 5310
            },
            Falcon500: {
                nominalVoltageVolts: 12,
                stallTorqueNewtonMeters: 4.69,
                stallCurrentAmps: 257,
                freeCurrentAmps: 1.5,
                freeSpeedRPM: 6380
            },
            MiniCIM: {
                nominalVoltageVolts: 12,
                stallTorqueNewtonMeters: 1.41,
                stallCurrentAmps: 89,
                freeCurrentAmps: 3,
                freeSpeedRPM: 5840
            },
            NEO: {
                nominalVoltageVolts: 12,
                stallTorqueNewtonMeters: 2.6,
                stallCurrentAmps: 105,
                freeCurrentAmps: 1.8,
                freeSpeedRPM: 5676
            },
            NEO550: {
                nominalVoltageVolts: 12,
                stallTorqueNewtonMeters: 0.97,
                stallCurrentAmps: 100,
                freeCurrentAmps: 1.4,
                freeSpeedRPM: 11000
            },
            RomiBuiltin: {
                nominalVoltageVolts: 4.5,
                stallTorqueNewtonMeters: 0.1765,
                stallCurrentAmps: 1.25,
                freeCurrentAmps: 0.13,
                freeSpeedRPM: 150
            },
            Vex775Pro: {
                nominalVoltageVolts: 12,
                stallTorqueNewtonMeters: 0.71,
                stallCurrentAmps: 134,
                freeCurrentAmps: 0.7,
                freeSpeedRPM: 18730
            },
        };
        let spec = specs[motorType];
        return `
            controllerType IS controllerType
            port IS port
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
