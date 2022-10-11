package provaserial;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.Scanner;
public class ProvaSerial {

    public static void main(String[] args) throws IOException {
        Scanner input = new Scanner(System.in);
        
        while(true){
        String inputParams[] = {"",""};
        
        while(inputParams[0].equals("")){
            System.out.println("Insert azimuth: ");
            inputParams[0] = input.nextLine();
        }
        
        while(inputParams[1].equals("")){
            System.out.println("Insert elevation: ");
            inputParams[1] = input.nextLine();
        }
        
        ProcessBuilder builder = new ProcessBuilder("python", 
                System.getProperty("user.dir") + "\\Python\\prova_serial.py", inputParams[0], inputParams[1]);
        try{
            Process process = builder.start();
            BufferedReader reader = new BufferedReader(new InputStreamReader(process.getInputStream()));
            BufferedReader error = new BufferedReader(new InputStreamReader(process.getErrorStream()));
            
            String lines = null;
            
            while((lines = reader.readLine()) != null && !lines.equals("END")){
                System.out.println(lines);
            }
            
            String errors = null;
            
            while((errors = error.readLine()) != null){
                System.out.println(errors);
            }
            
            process.destroy();
        }
        catch(IOException e){
            System.out.println(e.getMessage());
        }
    }
    }
    
}
