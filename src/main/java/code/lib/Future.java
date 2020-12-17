package code.lib;

import java.lang.reflect.InvocationTargetException;
import java.util.concurrent.ConcurrentLinkedDeque;

import org.mockito.Mockito;
import org.mockito.invocation.InvocationOnMock;
import org.mockito.stubbing.Answer;
import org.team199.wpiws.Pair;

/**
 * Represents an object which has not yet been created
 * @param T the object type contained in this Future
 */
public class Future<T> implements Answer<Object> {

    private T object;
    private final T forwarder;
    private final ConcurrentLinkedDeque<Pair<InvocationOnMock, Future<Object>>> invocations = new ConcurrentLinkedDeque<>();

    /**
     * Creates a new Future associated with no object
     * @param type the object type contained in this Future 
     */
    public Future(Class<? extends T> type) {
        object = null;
        forwarder = Mockito.mock(type, this);
    }

    /**
     * Retrieves an object associated with this Future. NOTE: all field references and final / non-public methods have undefined behavior on this object.
     * If this future has not yet been associated with an object, all method calls on this object return <code>null</code>.
     * If a method is called on this object and is future is associated with an object, the method call is forwarded to this Future's object. Otherwise, it is stored until this Future is associated with an object.
     * @return an object of type <code>T</code>
     * @see #set(Object)
     */
    public T get() {
        return forwarder;
    }

    /**
     * Sets the object associated with this Future
     * @param obj the object to associate with this Future
     * @throws FutureException if an Exception occures when calling a queued method on the object. If an exception occurs, all queued method calls may not be processed.
     * @see #get()
     */
    public void set(T obj) throws FutureException {
        synchronized(this) {
            object = obj;
            Pair<InvocationOnMock, Future<Object>> invocation;
            while((invocation = invocations.poll()) != null) {
                try {
                    Object result = invocation.val1.getMethod().invoke(object, invocation.val1.getArguments());
                    if(invocation.val2 != null) {
                        invocation.val2.set(result);
                    }
                } catch(InvocationTargetException e) {
                    throw new FutureException(e.getTargetException());
                } catch(Exception e) {
                    throw new FutureException(e);
                }
            }
        }
    }

    @Override
    public Object answer(InvocationOnMock invocation) throws Throwable {
        synchronized(this) {
            if(object == null) {
                if(invocation.getMethod().getReturnType().equals(Void.TYPE)) {
                    invocations.add(new Pair<>(invocation, null));
                    return null;
                } else {
                    Future<Object> result = new Future<>(invocation.getMethod().getReturnType());
                    invocations.add(new Pair<>(invocation, result));
                    return result;
                }
            } else {
                try {
                    return invocation.getMethod().invoke(object, invocation.getArguments());
                } catch(InvocationTargetException e) {
                    throw e.getTargetException();
                }
            }
        }
    }

}
