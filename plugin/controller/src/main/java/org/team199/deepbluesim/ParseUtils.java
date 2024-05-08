package org.team199.deepbluesim;

public final class ParseUtils {

    public static final double parseDoubleOrDefault(String str, double defaultValue) {
        try {
            return Double.parseDouble(str);
        } catch(NumberFormatException e) {
            return defaultValue;
        }
    }

    public static final int parseIntOrDefault(String str, int defaultValue) {
        try {
            return Integer.parseInt(str);
        } catch(NumberFormatException e) {
            return defaultValue;
        }
    }

}
