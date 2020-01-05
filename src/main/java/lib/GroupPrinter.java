package lib;

import java.util.ArrayList;
import java.util.function.Consumer;

public class GroupPrinter {
    private static GroupPrinter INSTANCE = new GroupPrinter();

    public GroupPrinter getInstance() {
        return INSTANCE;
    }
    ArrayList<String> toPrint = new ArrayList<>();


    public synchronized void addEntry(double timeStamp, String str) {
        if(str != null) {
            toPrint.add(timeStamp + ": " + str);
        } else {
            toPrint.add(timeStamp + ": String was null!");
        }
    }

    public synchronized void print(Consumer<String> print) {
        for (String str : toPrint) {
            print.accept(str);
        }
        toPrint.clear();
    }
}