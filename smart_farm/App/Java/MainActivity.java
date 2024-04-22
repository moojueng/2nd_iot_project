package com.example.test2;

import android.app.ProgressDialog;
import android.content.Intent;
import android.graphics.Color;
import android.os.AsyncTask;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.TextView;

import androidx.appcompat.app.AppCompatActivity;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.net.HttpURLConnection;
import java.net.URL;
import java.util.ArrayList;


public class MainActivity extends AppCompatActivity {

    private static String IP_ADDRESS = "192.168.1.226";
    private static String TAG = "phptest";

    private String mJsonString;

    private TextView S_val;
    private TextView T_val;
    private TextView H_val;
    private TextView C_val;
    private TextView W_val;
    private TextView W_title;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        MainActivity.GetData task = new MainActivity.GetData();
        task.execute( "http://" + IP_ADDRESS + "/farm_data.php", "");

        Button button_all = (Button) findViewById(R.id.chart_button);
        button_all.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Intent intent_flowchart = new Intent(MainActivity.this, FlowChart.class);
                startActivity(intent_flowchart);
            }
        });

    }

    private class GetData extends AsyncTask<String, Void, String> {

        ProgressDialog progressDialog;
        String errorString = null;

        @Override
        protected void onPreExecute() {
            super.onPreExecute();

            progressDialog = ProgressDialog.show(MainActivity.this,
                    "Please Wait", null, true, true);
        }


        @Override
        protected void onPostExecute(String result) {
            super.onPostExecute(result);

            progressDialog.dismiss();
            Log.d(TAG, "response - " + result);

            if (result == null){

            }
            else {

                mJsonString = result;
                showResult();
            }
        }


        @Override
        protected String doInBackground(String... params) {

            String serverURL = params[0];
            String postParameters = params[1];


            try {

                URL url = new URL(serverURL);
                HttpURLConnection httpURLConnection = (HttpURLConnection) url.openConnection();


                httpURLConnection.setReadTimeout(5000);
                httpURLConnection.setConnectTimeout(5000);
                httpURLConnection.setRequestMethod("POST");
                httpURLConnection.setDoInput(true);
                httpURLConnection.connect();


                OutputStream outputStream = httpURLConnection.getOutputStream();
                outputStream.write(postParameters.getBytes("UTF-8"));
                outputStream.flush();
                outputStream.close();


                int responseStatusCode = httpURLConnection.getResponseCode();
                Log.d(TAG, "response code - " + responseStatusCode);

                InputStream inputStream;
                if(responseStatusCode == HttpURLConnection.HTTP_OK) {
                    inputStream = httpURLConnection.getInputStream();
                }
                else{
                    inputStream = httpURLConnection.getErrorStream();
                }


                InputStreamReader inputStreamReader = new InputStreamReader(inputStream, "UTF-8");
                BufferedReader bufferedReader = new BufferedReader(inputStreamReader);

                StringBuilder sb = new StringBuilder();
                String line;

                while((line = bufferedReader.readLine()) != null){
                    sb.append(line);
                }

                bufferedReader.close();

                return sb.toString().trim();


            } catch (Exception e) {

                Log.d(TAG, "GetData : Error ", e);
                errorString = e.toString();

                return null;
            }

        }
    }


    private void showResult(){

        String TAG_JSON="data";
        String TAG_CDS = "cds";
        String TAG_SOIL = "soil";
        String TAG_WAR ="war";
        String TAG_TEM = "tem";
        String TAG_HUM = "hum";
        String TAG_DATE ="date";

        S_val = (TextView) findViewById(R.id.soil_view);
        T_val = (TextView) findViewById(R.id.tem_view);
        H_val = (TextView) findViewById(R.id.hum_view);
        C_val = (TextView) findViewById(R.id.cds_view);
        W_val = (TextView) findViewById(R.id.war_view);
        W_title = (TextView) findViewById(R.id.textView8);

        try {
            JSONObject jsonObject = new JSONObject(mJsonString);
            JSONArray jsonArray = jsonObject.getJSONArray(TAG_JSON);

            for(int i=0;i<jsonArray.length();i++){

                JSONObject item = jsonArray.getJSONObject(i);

                String cds = item.getString(TAG_CDS);
                String soil = item.getString(TAG_SOIL);
                String hum = "습도\n" + item.getString(TAG_HUM) + "%";
                String war = item.getString(TAG_WAR);
                String tem = "온도\n" + item.getString(TAG_TEM) + "°C";
                String date = item.getString(TAG_DATE);


                S_val.setText(soil);
                T_val.setText(tem);
                H_val.setText(hum);
                C_val.setText(cds);
                W_val.setText(war);
                int water = Integer.parseInt(war);
                if(water<=200) {
                    W_val.setBackgroundColor(Color.parseColor("#FFFF0000"));
                    W_title.setBackgroundColor(Color.parseColor("#FFFF0000"));
                }
                else if(water<400 && water > 200)
                {
                    W_val.setBackgroundColor(Color.parseColor("#FF002AFF"));
                    W_title.setBackgroundColor(Color.parseColor("#FF002AFF"));
                }
                else
                {
                    W_val.setBackgroundColor(Color.parseColor("#8BC34A"));
                    W_title.setBackgroundColor(Color.parseColor("#8BC34A"));
                }
            }



        } catch (JSONException e) {

            Log.d(TAG, "showResult : ", e);
        }

    }

}
