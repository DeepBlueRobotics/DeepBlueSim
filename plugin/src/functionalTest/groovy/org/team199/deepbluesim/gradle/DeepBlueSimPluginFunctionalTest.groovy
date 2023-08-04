/*
 * This Groovy source file was generated by the Gradle 'init' task.
 */
package org.team199.deepbluesim.gradle

import spock.lang.Specification
import org.gradle.testkit.runner.GradleRunner

import org.apache.commons.io.FileUtils


/**
 * A simple functional test for the 'org.team199.deepbluesim' plugin.
 */
class DeepBlueSimPluginFunctionalTest extends Specification {
    def "installDeepBlueSim task works"() {
        given:
        def projectDir = new File("build/functionalTest/installwebotsFolder")
        FileUtils.deleteDirectory(projectDir)
        projectDir.mkdirs()
        new File(projectDir, "settings.gradle") << ""
        new File(projectDir, "build.gradle") << """
            plugins {
                id('org.team199.deepbluesim')
            }
        """
        def webotsZipFile = new File(projectDir, "build/tmp/deepbluesim/Webots.zip")
        def webotsDir = new File(projectDir, "Webots")
        def webotsControllerJar = new File(projectDir, "Webots/controllers/DeepBlueSim/DeepBlueSim.jar")

        when:
        def runner = GradleRunner.create()
        runner.forwardOutput()
        runner.withPluginClasspath()
        runner.withArguments("installDeepBlueSim")
        runner.withProjectDir(projectDir)
        def result = runner.build()

        then:
        webotsZipFile.exists()
        webotsControllerJar.exists()
    }
}