package src;

import java.nio.file.attribute.DosFileAttributeView;
import java.util.*;

import javax.lang.model.util.ElementScanner6;

import src.API;

public class Main {
    static ArrayList<Node> s = new ArrayList<Node>();
    static ArrayList<Node> steps = new ArrayList<Node>();
    static int finished  = 0;

    private static void log(String text) {
        System.err.println(text);
    }

    public static void main(String[] args) {
        int[][] arr = new int[16][16];

        for (int i = 0; i < 16; i++) {
            for (int j = 0; j < 16; j++) {
                arr[i][j] = 0;
            }
        }

        int[][] weight = new int[16][16];

        for (int i = 0; i < weight.length / 2; i++) {
            for (int j = 0; j < weight.length / 2; j++) {
                weight[i][j] = (weight.length - 2 - i - j);
                weight[i][weight.length - 1 - j] = weight[i][j];
                weight[weight.length - 1 - i][j] = weight[i][j];
                weight[weight.length - 1 - i][weight.length - 1 - j] = weight[i][j];
            }
        }
        
        Main.log("Running...");
        
          for (int[] row : weight) {
          Main.log(Arrays.toString(row));
          }
         

        // API.setColor(0, 0, 'G');
        // API.setText(0, 0, "abc");
        // int x = findMinBetweenNeighbors(weight,15,15);
        //API.setColor(7, 7, 'r');
        
        try1(weight, 15, 0, 'u', ' ');
        /*
         * Node n = new Node(0, 0, 5);
         * Node n1 = new Node(1, 1, 4);
         * Node n2 = new Node(1, 1, 3);
         * Node n3 = new Node(1, 1, 4);
         * Node n4 = new Node(1, 1, 3);
         * Node n5 = new Node(1, 1, 2);
         * Node n6 = new Node(1, 1, 3);
         * Node n7 = new Node(1, 1, 2);
         * Node n8 = new Node(1, 1, 1);
         * Node n9 = new Node(1, 1, 0);
         * s.add(n);
         * s.add(n1);
         * s.add(n2);
         * s.add(n3);
         * s.add(n4);
         * s.add(n5);
         * s.add(n6);
         * s.add(n7);
         * s.add(n8);
         * s.add(n9);
         * int temp = 0;
         * for (int i = s.size() - 1; i >= 0; i--) {
         * s.get(i).setValue(temp);
         * temp++;
         * 
         * }
         * 
         * System.out.println(s.toString());
         */

    }

    // x+1 = down
    // x-1 = up
    // y+1 = right
    // y-1 = left

    public static void try1(int weight[][], int x, int y, char Facing, char prevF) {
        if (x < 0 || x > 15 || y < 0 || y > 15) {
            return;
        }
        if(finished == 1){
            return;
        }
        Main.log(String.valueOf(weight[x][y]));
        if(weight[x][y] == 0){
            
           // API.setColor(x, y, 'r');
           // API.setText(y, x, "MY BOI");
            //Main.log("FINISHED----------------------");
            //Main.log("x= " + String.valueOf(x) + "| y = "+ String.valueOf(y));
            finished = 1;
        }
        
        
        for(Node k : steps){
            if(x== k.getX() && y == k.getY()){
                Main.log("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
                backtrack(Facing, prevF);
                return;
            }
        }
        Node n = new Node(x,y,0);
        steps.add(n);
       /* for(Node k : steps){
            Main.log(k.toString());
        }
        */
        Main.log("__________________________________________________________________");
        Main.log(" x =  " + String.valueOf(x) + " y = " + String.valueOf(y));
        Main.log("Facing : " + Facing);
        Main.log("Prev : " + prevF);
        /*
         * int min = findMinBetweenNeighbors(weight, x, y); // should add it returns
         * also the x and y of that minimum
         * int temp = min;
         * API.setText(15-x, 15-y, String.valueOf(temp));
         * 
         * Node n = new Node(x, y, min + 1);
         * s.add(n);
         * for (int i = s.size() - 1; i >= 0; i--) {
         * s.get(i).setValue(temp);
         * temp++;
         * 
         * }
         */
        // should fix this to use with findMin between neighbors.
        int k = 0;
        switch (Facing) {
            case 'u':
                // we will only move forward // check for wall forward.
                if (!API.wallFront()) {
                    API.moveForward();
                    k++;
                }
                break;
            case 'd':
                // we will turn 180 and move // check for wall behind. // this is for
                // backtracking we will x this out i think
                // API.turnLeft();
                // API.turnLeft();
                // API.moveForward();
                break;
            case 'r':
                if (!API.wallLeft()) {
                    API.turnLeft();
                    API.moveForward();
                    k++;
                }
                // turn left + forward // check for wall left.
                break;
            case 'l':
                // r + f // check for wall right.
                if (!API.wallRight()) {
                    API.turnRight();
                    API.moveForward();
                    k++;
                }
                break;
        }
        Main.log("K  for up = " + String.valueOf(k));
        if (k != 0) {
            try1(weight, x - 1, y, 'u', Facing); // up
            if(finished == 1){
                return;
            }
        }

        k = 0;
        switch (Facing) {
            case 'u':
                // L L F // this is for backtracking i think
                // API.turnLeft();
                // API.turnLeft();
                // API.moveForward();
                break;
            case 'd':
                // F // check for wall forward
                if (!API.wallFront()) {
                    API.moveForward();
                    k++;
                }
                break;
            case 'r':
                // R F //check for wall Right
                if (!API.wallRight()) {
                    API.turnRight();
                    API.moveForward();
                    k++;
                }

                break;
            case 'l':
                // L F // check for wall left
                if (!API.wallLeft()) {
                    API.turnLeft();
                    API.moveForward();
                    k++;
                }

                break;
        }
        Main.log("K  for down = " + String.valueOf(k));
        if (k != 0) {

            try1(weight, x + 1, y, 'd', Facing); // down
            if(finished == 1){
                return;
            }
        }

        k = 0;

        switch (Facing) {
            case 'u':
                // R F // check for wall right
                if (!API.wallRight()) {
                    API.turnRight();
                    API.moveForward();
                    k++;
                }

                break;
            case 'd':
                // L F //check for wall right
                if (!API.wallLeft()) {
                    API.turnLeft();
                    API.moveForward();
                    k++;
                }

                break;
            case 'r':
                // F // check for wall ahead
                if (!API.wallFront()) {
                    API.moveForward();
                    k++;
                }
                break;
            case 'l':
                // R R F // this is back tracking
                // API.turnRight();
                // API.turnRight();
                // API.moveForward();
                break;
        }
        Main.log("" + Facing);
        Main.log("K  for right = " + String.valueOf(k));
        if (k != 0) {
            Main.log("GOING RIGHT");
            Main.log("new x =" + String.valueOf(x));
            Main.log("new y =" + String.valueOf(y + 1));
            Main.log("new facing = r ");
            Main.log("old facing = " + Facing);

            try1(weight, x, y + 1, 'r', Facing); // right
            if(finished == 1){
                return;
            }

        }

        k = 0;
        switch (Facing) {
            case 'u':
                // L F //check for wall left
                if (!API.wallLeft()) {
                    API.turnLeft();
                    API.moveForward();
                    k++;
                }

                break;
            case 'd':
                // L F // check for wall left
                if (!API.wallRight()) {
                    API.turnRight();
                    API.moveForward();
                    k++;
                }

                break;
            case 'r':
                // L L F // BackTracking
                // API.turnLeft();
                // API.turnLeft();
                // API.moveForward();
                break;
            case 'l':
                // F // check for wall forward.
                if (!API.wallFront()) {
                    API.moveForward();
                    k++;
                }

                break;
        }
        Main.log("K  for left = " + String.valueOf(k));
        if (k != 0) {
            try1(weight, x, y - 1, 'l', Facing); // left
            if(finished == 1){
                return;
            }
        }
        k = 0;

        // backtrack
        Main.log("BackTrack");
        //steps.remove(n);
        backtrack(Facing, prevF);
        // go oppsiite of the stack popping or go oppisite of the current facing . the
        // latter is better and easier.
    }

    private static void backtrack(char facing, char prev) {
        Main.log("########################################");
        Main.log("BackTracking");
        Main.log("Facing = " + facing);
        Main.log("previous = " + prev);
     

        if (Character.compare(prev, 'u') == 0) {
            switch (facing) {
                case 'u':
                    // L L F L L
                    API.turnLeft();
                    API.turnLeft();
                    API.moveForward();
                    API.turnLeft();
                    API.turnLeft();
                    break;

                case 'r':
                    // L L F R
                    API.turnLeft();
                    API.turnLeft();
                    API.moveForward();
                    API.turnRight();
                    break;
                case 'd':
                    // L L F
                    API.turnLeft();
                    API.turnLeft();
                    API.moveForward();
                    break;
                case 'l':
                    // L L F L
                    API.turnLeft();
                    API.turnLeft();
                    API.moveForward();
                    API.turnLeft();
                    break;
            }
        }

        if (Character.compare(prev, 'd') == 0) {
            switch (facing) {
                case 'u':
                    // L L F
                    API.turnLeft();
                    API.turnLeft();
                    API.moveForward();
                    break;

                case 'r':
                    // L L F L
                    API.turnLeft();
                    API.turnLeft();
                    API.moveForward();
                    API.turnLeft();
                    break;
                case 'd':
                    // L L F L L
                    API.turnLeft();
                    API.turnLeft();
                    API.moveForward();
                    API.turnLeft();
                    API.turnLeft();
                    break;
                case 'l':
                    // L L F R
                    API.turnLeft();
                    API.turnLeft();
                    API.moveForward();
                    API.turnRight();
                    break;
            }
        }
        if (Character.compare(prev, 'r') == 0) {
            switch (facing) {
                case 'u':
                    // L L F R
                    API.turnLeft();
                    API.turnLeft();
                    API.moveForward();
                    API.turnLeft();
                    break;

                case 'r':
                    // L L F L L
                    API.turnLeft();
                    API.turnLeft();
                    API.moveForward();
                    API.turnLeft();
                    API.turnLeft();
                    break;
                case 'd':
                    // L L F L
                    API.turnLeft();
                    API.turnLeft();
                    API.moveForward();
                    API.turnRight();
                    break;
                case 'l':
                    // L L F
                    API.turnLeft();
                    API.turnLeft();
                    API.moveForward();
                    break;
            }
        }
        if (Character.compare(prev, 'l') == 0) {
            switch (facing) {
                case 'u':
                    // L L F R
                    API.turnLeft();
                    API.turnLeft();
                    API.moveForward();
                    API.turnRight();
                    break;

                case 'r':
                    // L L F
                    API.turnLeft();
                    API.turnLeft();
                    API.moveForward();
                    break;
                case 'd':
                    // L L F L
                    Main.log("Prev = " + prev);
                    Main.log("Current = " + facing);
                    API.turnLeft();
                    API.turnLeft();
                    API.moveForward();
                    API.turnLeft();

                    break;
                case 'l':
                    // L L F L L
                    API.turnLeft();
                    API.turnLeft();
                    API.moveForward();
                    API.turnLeft();
                    API.turnLeft();
                    break;
            }
        }

        Main.log("finished BackTracking");
    }

    private static int findMinBetweenNeighbors(int weight[][], int x, int y) {
        int value = 99;
        // fix this with what way we are facing
        if (!API.wallFront() && x < 15) { // up
            if (value < weight[x + 1][y]) {
                value = weight[x + 1][y];

            }

        }
        if (x > 0) { // down
            Main.log(String.valueOf(weight[x - 1][y]));
            if (value < weight[x - 1][y]) {
                value = weight[x - 1][y];

            }
        }

        if (!API.wallRight() && y < 15) { // right
            if (value < weight[x][y + 1]) {
                value = weight[x][y + 1];
            }
        }

        if (!API.wallRight() && y > 0) { // right
            if (value < weight[x][y - 1]) {
                value = weight[x][y - 1];
            }
        }

        return value;
    }

}
