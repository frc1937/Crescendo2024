package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.net.HttpURLConnection;
import java.net.URL;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

public class Vision extends SubsystemBase {

    // Replace this URL with the actual URL of the JSON API
    private static final String API_URL = "http://127.0.0.1:5000/vision";
    private double distance;
    private double angle;

    public Vision() {
        // Initialize the subsystem here
    }

    public void updateVisionData() {
        try {
            // Create a URL object from the API URL
            URL url = new URL(API_URL);

            // Open a connection to the URL
            HttpURLConnection connection = (HttpURLConnection) url.openConnection();

            // Set the request method to GET
            connection.setRequestMethod("GET");

            // Get the response code
            int responseCode = connection.getResponseCode();

            // Check if the request was successful (HTTP 200 OK)
            if (responseCode == HttpURLConnection.HTTP_OK) {
                // Read the response from the API
                BufferedReader reader = new BufferedReader(new InputStreamReader(connection.getInputStream()));
                StringBuilder response = new StringBuilder();
                String line;

                while ((line = reader.readLine()) != null) {
                    response.append(line);
                }

                // Parse the JSON response
                JSONParser parser = new JSONParser();
                JSONArray jsonArray = (JSONArray) parser.parse(response.toString());

                // Process each object in the JSON array
                for (Object obj : jsonArray) {
                    if (obj instanceof JSONObject) {
                        JSONObject jsonObject = (JSONObject) obj;

                        // Extract information from the JSON object
                        // String className = (String) jsonObject.get("class_name");
                        distance = (double) jsonObject.get("distance");
                        angle = (double) jsonObject.get("angle");
                    }
                }

                // Close the reader
                reader.close();
            } else {
            }

            // Disconnect the connection
            connection.disconnect();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public double getDistance() {
        return distance;
    }

    public double getAngle() {
        return angle;
    }
}
