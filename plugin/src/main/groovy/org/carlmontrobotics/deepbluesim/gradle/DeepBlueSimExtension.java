package org.carlmontrobotics.deepbluesim.gradle;

import java.io.File;
import java.nio.ByteBuffer;
import java.nio.channels.FileChannel;
import java.nio.channels.FileLock;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;
import java.util.Calendar;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.concurrent.Semaphore;
import java.util.stream.Stream;

import javax.inject.Inject;

import org.gradle.api.Action;
import org.gradle.api.Project;
import org.gradle.api.Task;
import org.gradle.api.file.DirectoryProperty;
import org.gradle.api.file.FileTree;
import org.gradle.api.provider.Provider;
import org.gradle.internal.os.OperatingSystem;
import org.gradle.process.JavaForkOptions;

import edu.wpi.first.gradlerio.wpi.WPIExtension;
import edu.wpi.first.gradlerio.wpi.java.ExtractNativeJavaArtifacts;
import edu.wpi.first.gradlerio.wpi.java.TestTaskDoFirstAction;
import edu.wpi.first.gradlerio.wpi.simulation.HalSimExtension;
import edu.wpi.first.gradlerio.simulation.HalSimPair;

/** A Gradle extension that allows tasks to be configured so that they can use WPILib's simulation extensions. */
public class DeepBlueSimExtension {

    private Project project;
    private WPIExtension wpi;

    /**
     * Constructs an instance for use with a project and a WPI extension.
     * @param project the project that this will be used with
     * @param wpi the WPI extension to use when configuring a task
     */
    @Inject
    public DeepBlueSimExtension(Project project, WPIExtension wpi) {
        this.project = project;
        this.wpi = wpi;
    }

    /**
     * Returns a dependency that should be added to any test configuration that wants to work
     * effectively with DeepBlueSim.
     * 
     * @return the test library dependency
     */
    public FileTree testDep() {
        return project.fileTree("libdeepbluesim");
    }
    
    /** 
     * Configures the tasks so that they can use the WPILib simulation extensions that are enabled by default.
     *  
     * @param tasks      the tasks to be configured
     * @throws Exception if the tasks can not be configured
     */
    public void configureForSimulation(List<Task> tasks) throws Exception {
        configureForSimulation(tasks, null);
    }

    /**
     * Configures a task so it can use the WPILib simulation extensions that are enabled by default.
     * 
     * @param t          the task to be configured
     * @throws Exception if the task can't be configured
     */
    public void configureForSimulation(Task t) throws Exception {
        configureForSimulation(t, null);
    }

    /**
     * Configures the tasks so they can use the specificed WPILib simulation extensions.
     * 
     * @param tasks      the tasks to be configured
     * @param extensions the extensions to use. If null, the extensions that are enabled by default are used.
     * @throws Exception if the task can't be configured
     */
    public void configureForSimulation(List<Task> tasks, List<HalSimExtension> extensions) throws Exception {
        for (var t : tasks) {
            configureForSimulation(t, extensions);
        }
    }

    // DeepBlueSim uses WPI's HAL sim websocket server extension and only one process can be started
    // with that extension at a time, presumably because it listens on a TCP port. Gradle can create
    // multiple processes each with multiple threads each with different instances of this class. To
    // ensure only one DeepBlueSim test task is running at a time, we use a file lock to ensure
    // there is only one gradle process and a semaphore to ensure there is only one thread within
    // that process that is running a DeepBlueSim test task.
    private static Semaphore semaphore = new Semaphore(1);
    private static volatile FileLock fileLock = null;

    /**
     * Configures a task so it can use the specificed WPILib simulation extensions.
     * 
     * @param t          the task to be configured
     * @param extensions the extensions to use. If null, use the extensions that are enabled by default.
     * @throws Exception if the task can't be configured
     */
    public void configureForSimulation(Task t, List<HalSimExtension> extensions) throws Exception {
        var extNames = extensions != null ? extensions.stream().map(x -> x.getName()).toList() : null;
        var java = wpi.getJava();
        var sim = wpi.getSim();
        boolean debug = java.getDebugJni().get().booleanValue();
        var extract = debug ? java.getExtractNativeDebugArtifacts() : java.getExtractNativeReleaseArtifacts();
        configureExecutableNatives(t, extract);
        if (OperatingSystem.current().isMacOsX()) {
            // Call t.jvmArgs("-XstartOnFirstThread") if that method exists
            try {
                java.lang.reflect.Method jvmArgsMethod = t.getClass().getMethod("jvmArgs", String.class);
                jvmArgsMethod.invoke(t, "-XstartOnFirstThread");
            } catch (NoSuchMethodException nsmEx) {
                // Ignored
            }
        }

        t.doFirst(new Action<Task>() {

            @Override
            public void execute(Task task) {
                try {
                    var logger = project.getLogger();
                    File ldpath = extract.get().getDestinationDirectory().get().getAsFile();
                    List<HalSimPair> halSimPairs = sim.getHalSimLocations(List.of(ldpath), debug);
                    Map<String, ?> env = sim.getEnvironment();

                    // Call t.environment(env) if that method exists
                    t.getClass().getMethod("environment", Map.class).invoke(t, env);

                    Stream<HalSimPair> extensionStream;
                    if (extNames != null) {
                        extensionStream = halSimPairs.stream().filter(x -> extNames.contains(x.name));
                    } else {
                        extensionStream = halSimPairs.stream().filter(x -> x.defaultEnabled);
                    }
                    Optional<String> extensionString = extensionStream.map(x -> x.libName).reduce((a, b) -> a + File.pathSeparator + b);

                    if (extensionString.isPresent()) {
                        // Call t.environment("HALSIM_EXTENSIONS", extensionString.get()) if that method exists
                        t.getClass().getMethod("environment", String.class, Object.class).invoke(t, "HALSIM_EXTENSIONS", extensionString.get());
                    }

                    if (OperatingSystem.current().isWindows()) {
                        System.out.println(
                                "If you receive errors loading the JNI dependencies, make sure you have the latest Visual Studio C++ Redstributable installed.");
                        System.out.println(
                                "That can be found at https://support.microsoft.com/en-us/help/2977003/the-latest-supported-visual-c-downloads");
                    }
                    logger.debug("deepbluesim is waiting for semaphore");
                    semaphore.acquire();
                    logger.debug("deepbluesim acquired semaphore");
                    if (fileLock == null) {
                        var lockFilePath =
                                Path.of(System.getProperty("java.io.tmpdir"),
                                        "deepbluesim.lock");
                        var channel = FileChannel.open(lockFilePath,
                                StandardOpenOption.CREATE,
                                StandardOpenOption.WRITE,
                                StandardOpenOption.TRUNCATE_EXISTING);
                        fileLock = channel.tryLock();
                        while (fileLock == null) {
                            logger.debug(
                                    "deepbluesim is waiting for file lock. See {}",
                                    lockFilePath);
                            Thread.sleep(10000);
                            fileLock = channel.tryLock();
                        }
                        logger.debug("deepbluesim acquired file lock.");
                        // The lock will be released when the JVM exits.
                        channel.write(
                                ByteBuffer.wrap("Locked by deepbluesim %Tc"
                                        .formatted(Calendar.getInstance())
                                        .getBytes()));
                    }
                } catch (Exception ex) {
                    throw new RuntimeException(ex);
                }
            }
        });
        t.doLast(new Action<Task>() {

            @Override
            public void execute(Task task) {
                var logger = project.getLogger();
                logger.debug("deepbluesim is releasing semaphore");
                try {
                    logger.debug("deepbluesim is releasing file lock");
                    fileLock.release();
                } catch (Exception ex) {
                    throw new RuntimeException(ex);
                } finally {
                    fileLock = null;
                }
                semaphore.release();
            }
        });

    }

    private void configureExecutableNatives(Task tt, Provider<ExtractNativeJavaArtifacts> extract) {
        tt.dependsOn(extract);

        Provider<DirectoryProperty> destDir = project.getProviders().provider(() -> {
            return extract.get().getDestinationDirectory();
        });

        tt.getInputs().dir(destDir);

        if (tt instanceof JavaForkOptions) {
            tt.doFirst(new TestTaskDoFirstAction((JavaForkOptions)tt, destDir));
        }
    }
}
