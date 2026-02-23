package frc.robot.utils.functional;

import java.util.Objects;
import java.util.function.Function;

@FunctionalInterface
public interface QuadFunction<T, U, V, W, R> {

    public abstract R apply(T t, U u, V v, W w);

    /** Composes this quadfunction with another function */
    public default <X> QuadFunction<T, U, V, W, X> andThen(Function<? super R, ? extends X> after) {
        Objects.requireNonNull(after);
        return (final T t, final U u, final V v, final W w) -> after.apply(this.apply(t, u, v, w));
    }
}
