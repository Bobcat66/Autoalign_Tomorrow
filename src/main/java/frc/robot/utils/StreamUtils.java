package frc.robot.utils;

import java.util.Iterator;
import java.util.function.BiFunction;
import java.util.stream.Stream;
import java.util.stream.StreamSupport;

import frc.robot.utils.functional.TriFunction;

public class StreamUtils {
    public static <A, B, C> Stream<C> zip(
        Stream<A> a,
        Stream<B> b,
        BiFunction<A, B, C> zipper
    ) {
        Iterator<A> itA = a.iterator();
        Iterator<B> itB = b.iterator();

        Iterable<C> iterable = () -> new Iterator<>() {
            public boolean hasNext() {
                return itA.hasNext() && itB.hasNext();
            }

            public C next() {
                return zipper.apply(itA.next(), itB.next());
            }
        };

        return StreamSupport.stream(iterable.spliterator(), false);
    }

    public static <A, B, C, D> Stream<D> trizip(
        Stream<A> a,
        Stream<B> b,
        Stream<C> c,
        TriFunction<A, B, C, D> trizipper
    ) {
        Iterator<A> itA = a.iterator();
        Iterator<B> itB = b.iterator();
        Iterator<C> itC = c.iterator();

        Iterable<D> iterable = () -> new Iterator<>() {
            public boolean hasNext() {
                return itA.hasNext() && itB.hasNext() && itC.hasNext();
            }

            public D next() {
                return trizipper.apply(itA.next(), itB.next(), itC.next());
            }
        };

        return StreamSupport.stream(iterable.spliterator(),false);
    }
}
