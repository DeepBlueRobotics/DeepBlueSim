package frc.robot;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Arrays;
import java.util.EnumSet;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;
import java.util.concurrent.atomic.AtomicInteger;

import org.junit.jupiter.api.Test;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;

public class NetworkTablesLoadTest {

    private AtomicInteger receivedCount = new AtomicInteger(0);

    @Test
    public void testClientRepublishesBeforeCloseFlushes()
            throws InterruptedException, ExecutionException, TimeoutException {
        var testTopicName = "testTopic";
        var count = 1000;
        CompletableFuture<Boolean> isDone = new CompletableFuture<>();
        var pubSubOptions = new PubSubOption[] {PubSubOption.sendAll(true),
                PubSubOption.keepDuplicates(true),
                PubSubOption.periodic(Double.MIN_VALUE)};
        var server = NetworkTableInstance.create();
        server.startServer();
        var serverTopic = server.getDoubleArrayTopic(testTopicName);
        var subscriber = serverTopic.subscribe(new double[] {}, pubSubOptions);
        server.addListener(subscriber, EnumSet.of(Kind.kValueRemote),
                (event) -> {
                    if (receivedCount.incrementAndGet() == count) {
                        isDone.complete(true);
                    }
                    // Warnings about duplicate pubs occur if I either introduce this short delay...
                    try {
                        Thread.sleep(1);
                    } catch (Exception ex) {
                        throw new RuntimeException(ex);
                    }
                    // ...or a little IO
                    // System.out.println("Got %d: %s"
                    // .formatted(receivedCount.get(), Arrays.toString(
                    // event.valueData.value.getDoubleArray())));
                });

        var client = NetworkTableInstance.create();
        client.setServer("localhost");
        var clientName = "test client";
        client.startClient4(clientName);
        Thread.sleep(2000); // Startup time.
        int sentCount = 0;
        while (sentCount < count) {
            var clientTopic = client.getDoubleArrayTopic(testTopicName);
            try (var publisher = clientTopic.publish(pubSubOptions)) {
                publisher.set(new double[] {sentCount, sentCount, sentCount});
                // client.flush();
                sentCount++;
            }
            Thread.yield();
        }
        assertTrue(isDone.get(10, TimeUnit.SECONDS));
    }
}
