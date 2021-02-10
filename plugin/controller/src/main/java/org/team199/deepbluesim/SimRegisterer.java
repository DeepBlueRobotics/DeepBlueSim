package org.team199.deepbluesim;

import org.team199.deepbluesim.mediators.*;

import org.team199.wpiws.ScopedObject;
import org.team199.wpiws.UniqueArrayList;
import org.team199.wpiws.devices.EncoderSim;
import org.team199.wpiws.devices.PWMSim;
import org.team199.wpiws.devices.SimDeviceSim;
import org.team199.wpiws.interfaces.SimDeviceCallback;

// Performs automatic registration of callbacks detecting both the initalization of new devices as well as data callbacks for devices such as Motors, Gyros, etc.
// This allows us to automatically link these devices to Webots, reducing the amount of code we would have to change from a standard robot project
public class SimRegisterer {
    
    private static final SimDeviceCallback MISC_DEVICE_CALLBACK = SimRegisterer::callback;
    private static final UniqueArrayList<ScopedObject<?>> CALLBACKS = new UniqueArrayList<>();

    static {
        // Register Initalized Callbacks for Misc Devices
        CALLBACKS.add(SimDeviceSim.registerDeviceCreatedCallback("", MISC_DEVICE_CALLBACK, true));
        // Register Initalized Callbacks for PWM Devices
        CALLBACKS.add(PWMSim.registerStaticInitializedCallback((name, isInitialized) -> {
            if(isInitialized) {
                callback("PWM", name, 0);
            }
        }, true));
        // Register Initalized Callbacks for Encoder Devices
        CALLBACKS.add(EncoderSim.registerStaticInitializedCallback((name, isInitialized) -> {
            if(isInitialized) {
                callback("Encoder", name, 0);
            }
        }, true));
    }

    // Initalize SimRegisterer. This method exists to ensure that the static block is called
    public static void init() {}

    // Callback methods which place new devices in a processing queue which is processed every robot period
    // The WPILib callbacks are notified as part of the device creation. This process ensures that the devices complete their setup process
    // This is especially important for SimDevice's because their initalized callbacks can be notified before their values have been created
    // Queuing also ensures that callbacks (which are usually executed asycronously) are processed syncronously with the rest of the robot code

    // Callback for when a Miscellaneous Device is registered
    private static void callback(String deviceName) {
        if(deviceName.startsWith("navX")) {
            // If a navX is registered, try to link its SimDevice to the Webots robot
            MockGyro.linkGyro();
        }
        // The check for forward slash is to only get the talon/victor sim device corresponding to the motor instead of
        // for the QuadEncoder, Analog In, Fwd Limit, and Rev Limit.
        else if ((deviceName.startsWith("Talon") || deviceName.startsWith("Victor")) && !deviceName.contains("/")) {
            MockPhoenixMotor.linkMotor(deviceName);
        }
        else if (deviceName.startsWith("SPARK")) {
            MockSparkMotor.linkMotor(deviceName);
        }
    }

    // Callback for when a known device type is registered on a Non-Can port
    private static void callback(String type, String port, int storePos) {
        if(type.equals("PWM")) {
            // If a new PWM device has been initalized, attempt to link it to a Webots Motor
            // Register a speed callback on this device
            CALLBACKS.add(new PWMSim(port).registerSpeedCallback(
                // Call a motor forwarder for a callback
                new WebotsMotorForwarder("PWM[" + port + "]"),
                // Initalize with current speed
                true));
        }
        else if(type.equals("Encoder")) {
            // If a new PWM device has been initalized, attempt to link it to a Webots Motor
            // Register a speed callback on this device
            EncoderSim sim = new EncoderSim(port);
            new MockedEncoder(CALLBACKS, sim, port);
        }
    }

}