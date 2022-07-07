package src;

public class delete {
   public static void main(String[] args) {
    char prev = 'd';
    char facing = 'd';
    switch (prev) {
        case 'u':
            switch (facing) {
                case 'u':
                    // L L F L L
                  //  API.turnLeft();
                   // API.turnLeft();
                   // API.moveForward();
                   // API.turnLeft();
                   // API.turnLeft();
                    break;

                case 'r':
                    // L L F R
                   // API.turnLeft();
                   // API.turnLeft();
                   // API.moveForward();
                   // API.turnRight();
                    break;
                case 'd':
                    // L L F
                  //  API.turnLeft();
                  //  API.turnLeft();
                   // API.moveForward();
                    break;
                case 'l':
                    // L L F L
                  //  API.turnLeft();
                  //  API.turnLeft();
                   // API.moveForward();
                   // API.turnLeft();
                    break;
            }
        case 'd':
            switch (facing) {
                case 'u':
                    // L L F
                  //  API.turnLeft();
                   // API.turnLeft();
                   // API.moveForward();
                    break;

                case 'r':
                    // L L F L
                   // API.turnLeft();
                  //  API.turnLeft();
                   // API.moveForward();
                   // API.turnLeft();
                    break;
                case 'd':
                    // L L F L L
                  //  API.turnLeft();
                  //  API.turnLeft();
                 //   API.moveForward();
                  //  API.turnLeft();
                  //  API.turnLeft();
                    System.out.println("dd");
                    break;
                case 'l':
                    // L L F R
                   // API.turnLeft();
                  //  API.turnLeft();
                   // API.moveForward();
                   // API.turnRight();
                    break;
            }
        case 'r':
            switch (facing) {
                case 'u':
                    // L L F R
 
                    break;

                case 'r':
                    // L L F L L
  
                    break;
                case 'd':
                    // L L F L
  
                    break;
                case 'l':
                    // L L F

                    break;
            }
        case 'l':
            switch (facing) {
                case 'u':
                    // L L F R

                    break;

                case 'r':
                    // L L F

                    break;
                case 'd':
                    // L L F L
    
                    break;
                case 'l':
                    // L L F L L
    
                    break;
            }

    }
   }
    
}
