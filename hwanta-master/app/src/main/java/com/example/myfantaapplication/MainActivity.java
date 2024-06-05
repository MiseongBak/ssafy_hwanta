package com.example.myfantaapplication;

import android.app.AlertDialog;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.DialogInterface;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.pm.PackageManager;
import android.location.Location;
import android.location.LocationListener;
import android.location.LocationManager;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.view.WindowManager;
import android.widget.Button;

import androidx.activity.EdgeToEdge;
import androidx.annotation.NonNull;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;
import androidx.core.graphics.Insets;
import androidx.core.view.ViewCompat;
import androidx.core.view.WindowInsetsCompat;

import android.Manifest;

import com.google.gson.Gson;
import com.google.gson.JsonObject;

import okhttp3.*;

import java.io.IOException;
import java.time.LocalTime;
import java.util.Objects;


public class MainActivity extends AppCompatActivity {
    private static int hour;
    private static int minute;
    private static int second;

    // GPS 사용을 위한 멤버 변수 선언
    LocationManager locationManager;

    public static int cancel_visible = 0;
    public static int cancel_visible2 = 0;


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        EdgeToEdge.enable(this);
        setContentView(R.layout.activity_main);

        locationManager = (LocationManager) getSystemService(LOCATION_SERVICE);

        Button moveButton = findViewById(R.id.button1);
        Button btn_cancel = findViewById(R.id.button2); //(1)

        if(cancel_visible == 1)
        {
            btn_cancel.setVisibility(View.VISIBLE);
        }
        else
        {
            btn_cancel.setVisibility(View.INVISIBLE);
        }

        Intent intent2 = getIntent();
        int changeVis = intent2.getIntExtra("changebtn", 0);
        if(changeVis == 1){
            btn_cancel.setVisibility(View.INVISIBLE);
        }
        moveButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                new AlertDialog.Builder(MainActivity.this)
                        .setMessage("허위 신고 시 처벌 받을 수 있습니다.\n그래도 호출하시겠습니까?")
                        .setPositiveButton("확인", new DialogInterface.OnClickListener() {
                            @Override
                            public void onClick(DialogInterface dialog, int which) {

                                LocalTime currentTime = LocalTime.now();

                                hour = currentTime.getHour();
                                minute = currentTime.getMinute();
                                second = currentTime.getSecond();
                                System.out.println("click 시간: " +hour + "시 " + minute + "분 " +second + "초");

                                // Inside onClick() method
                                double latitude = 0;
                                double longitude = 0;

                                if (ActivityCompat.checkSelfPermission(MainActivity.this, Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED && ActivityCompat.checkSelfPermission(MainActivity.this, Manifest.permission.ACCESS_COARSE_LOCATION) != PackageManager.PERMISSION_GRANTED) {
                                    // Request location permissions if not granted
                                    ActivityCompat.requestPermissions(MainActivity.this,
                                            new String[]{Manifest.permission.ACCESS_FINE_LOCATION, Manifest.permission.ACCESS_COARSE_LOCATION}, 100);
                                    return;
                                }

                                // Get the location
                                Location location = locationManager.getLastKnownLocation(LocationManager.GPS_PROVIDER);
                                if (location != null) {
                                    latitude = location.getLatitude();
                                    longitude = location.getLongitude();
                                } else {
                                    Location locations = locationManager.getLastKnownLocation(LocationManager.NETWORK_PROVIDER);
                                    if (locations != null) {
                                        latitude = locations.getLatitude();
                                        longitude = locations.getLongitude();
                                    }
                                }

                                // Call the sendPostRequest method with latitude and longitude
                                // sendPostRequest(latitude, longitude);
                                sendPostRequest(latitude, longitude);

                            }
                        })
                        .setNegativeButton("취소", new DialogInterface.OnClickListener() {
                            @Override
                            public void onClick(DialogInterface dialog, int which) {
                                dialog.dismiss();
                            }
                        })
                        .show();
            }
        });

        btn_cancel.setOnClickListener(new View.OnClickListener() { //(2)
            @Override
            public void onClick(View v) {


                LocalTime cancelTime = LocalTime.now();

                int chour = cancelTime.getHour() - hour;
                int cminute = cancelTime.getMinute() - minute;
                int csecond = cancelTime.getSecond() - second;

                if(chour < 0){
                    chour += 24;
                }
                if(cminute < 0) {
                    cminute += 60;
                }
                if(csecond < 0){
                    csecond += 60;
                }
                int flag = chour*3600 + cminute*60 + csecond;

                System.out.println(flag);
                if(flag > 10){
                    Intent intent=new Intent(getApplicationContext(), ImpossibleActivity.class);
                    startActivity(intent); //(3)

                }
                else{
                    Intent intent=new Intent(getApplicationContext(), CancelActivity.class);
                    startActivity(intent); //(3)
                    cancel_visible = 0;

                }
            }
        });

        Button btn_help = findViewById(R.id.button3); //(1)

        btn_help.setOnClickListener(new View.OnClickListener() { //(2)
            @Override
            public void onClick(View v) {
                Intent intent=new Intent(getApplicationContext(), HelpActivity.class);
                startActivity(intent); //(3)
            }
        });

        // 로케이션 매니저 참조 선언
        // locationManager = (LocationManager) this.getSystemService(LOCATION_SERVICE);

        // 사용자에게 GPS 사용 권한 요청
        if (ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION)!= PackageManager.PERMISSION_GRANTED
                && ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_COARSE_LOCATION) != PackageManager.PERMISSION_GRANTED) {

            ActivityCompat.requestPermissions(MainActivity.this,
                    new String[] {Manifest.permission.ACCESS_FINE_LOCATION, Manifest.permission.ACCESS_COARSE_LOCATION}, 100);
            return;
        }
        // 사용자에게 GPS 사용 권한을 허가 받으면 GPS 정보를 불러오는 메소드, requestLocationUpdates
        // requestLocationUpdates(String provider, long minTimeMs, float minDistanceM, LocationListener listener, Looper looper)
        // 첫번째 파라미터 : Provider 지정 -> LocationManager.GPS_PROVIDER (GPS 사용 선언)
        // 두번째 파라미터 : GPS 호출 시간 간격
        // 세번째 파라미터 : GPS 호출 최소 이동거리
        // 네번재 파라미터 : 로캐이션 리스너 이름

        WindowManager.LayoutParams layoutParams = new WindowManager.LayoutParams();
        layoutParams.flags = WindowManager.LayoutParams.FLAG_DIM_BEHIND;
        layoutParams.dimAmount = 0.8f;
        getWindow().setAttributes(layoutParams);


        ViewCompat.setOnApplyWindowInsetsListener(findViewById(R.id.main), (v, insets) -> {
            Insets systemBars = insets.getInsets(WindowInsetsCompat.Type.systemBars());
            v.setPadding(systemBars.left, systemBars.top, systemBars.right, systemBars.bottom);
            return insets;
        });
    }


        // 앱 권한 요청 설정 메소드
    @Override
    public void onRequestPermissionsResult(int requestCode, @NonNull String[] permissions, @NonNull int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);

        if (requestCode == 100) {
            // 권한 요청
            if (ActivityCompat.checkSelfPermission(MainActivity.this, Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED
                    && ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_COARSE_LOCATION) != PackageManager.PERMISSION_GRANTED) {
                ActivityCompat.requestPermissions(MainActivity.this,
                        new String[] {Manifest.permission.ACCESS_FINE_LOCATION, Manifest.permission.ACCESS_COARSE_LOCATION}, 100);
            }
        }
    }
    public void sendPostRequest(double latitude, double longitude) {
        OkHttpClient client = new OkHttpClient();

        // Form 데이터를 JSON 형식으로 변환
        JsonObject jsonBody = new JsonObject();
        jsonBody.addProperty("lat", String.valueOf(latitude));
        jsonBody.addProperty("lon", String.valueOf(longitude));
        String jsonString = new Gson().toJson(jsonBody);

        Log.d("MainActivity json", jsonString);

        // POST 요청 생성
        RequestBody requestBody = RequestBody.create(jsonString, MediaType.parse("application/json"));
        Request request = new Request.Builder()
                .url("https://j10c110.p.ssafy.io/api/patients/1")
                .put(requestBody)
                .build();

        // 요청 실행
        client.newCall(request).enqueue(new Callback() {
            @Override
            public void onResponse(Call call, Response response) throws IOException {
                try{
                    int statusCode = response.code();
                    if (response.isSuccessful()) {
                        // 요청 성공 시 처리
                        Log.d("MainActivity", "UPDATE request successful");

                        cancel_visible = 1;

                        Intent intent = new Intent(getApplicationContext(), CallActivity.class);
                        startActivity(intent);
                    } else {
                        // 요청 실패 시 처리
                        Log.d("MainActivity", "UPDATE request failed");
                        Log.d("MainActivity", String.valueOf(statusCode));

                        Intent intent = new Intent(getApplicationContext(), CallFailActivity.class);
                        startActivity(intent);
                    }
                } finally {
                    if(response != null){
                        response.close();
                    }
                }

            }

            @Override
            public void onFailure(Call call, IOException e) {
                // 요청 실패 시 처리
                Log.e("MainActivity", "Failed to send UPDATE request", e);

                Intent intent = new Intent(getApplicationContext(), CallFailActivity.class);
                startActivity(intent);
            }
        });
    }
}