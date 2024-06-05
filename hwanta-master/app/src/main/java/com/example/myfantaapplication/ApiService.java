package com.example.myfantaapplication;

import android.app.Service;
import android.content.Intent;
import android.os.Handler;
import android.os.IBinder;
import android.util.Log;
import android.view.View;
import android.widget.Button;

import androidx.annotation.Nullable;

import okhttp3.Call;
import okhttp3.Callback;
import okhttp3.OkHttpClient;
import okhttp3.Request;
import okhttp3.Response;

import java.io.IOException;

public class ApiService extends Service {

    private static final String TAG = "ApiService";
    private static final String API_URL = "https://j10c110.p.ssafy.io/api/arrive/1";
    private static final int POLL_INTERVAL = 1000; // 1초마다 호출

    private final OkHttpClient client = new OkHttpClient();
    private final Handler handler = new Handler();
    private boolean isRunning = false;

    @Override
    public int onStartCommand(Intent intent, int flags, int startId) {
        Log.d(TAG, "Service started");
        isRunning = true;
        startApiPolling();
        return START_STICKY;
    }

    @Override
    public void onDestroy() {
        super.onDestroy();
        Log.d(TAG, "Service destroyed");
        stopApiPolling();
    }

    @Nullable
    @Override
    public IBinder onBind(Intent intent) {
        return null;
    }

    private void startApiPolling() {
        handler.postDelayed(new Runnable() {
            @Override
            public void run() {
                callApi();
                if (isRunning) {
                    handler.postDelayed(this, POLL_INTERVAL);
                }
            }
        }, 0); // 초기에 한 번 호출
    }

    private void stopApiPolling() {
        isRunning = false;
        handler.removeCallbacksAndMessages(null);
    }

    private void callApi() {
        Request request = new Request.Builder()
                .url(API_URL)
                .build();

        client.newCall(request).enqueue(new Callback() {
            @Override
            public void onFailure(Call call, IOException e) {
                Log.e(TAG, "Failed to make GET request", e);
                // 응답 실패 시 처리
            }

            @Override
            public void onResponse(Call call, Response response) throws IOException {
                try (Response responseBody = response) {
                    if (!response.isSuccessful()) {
                        Log.e(TAG, "Unexpected response code: " + response);
                        return;
                    }

                    String responseData = responseBody.body().string();
                    // 예상 응답을 기반으로 처리
                    boolean isSuccess = responseData.equals("true");
                    handleResponse(isSuccess);
                }
            }
        });
    }

    private void handleResponse(boolean isSuccess) {
        // 응답에 따른 처리
        if (isSuccess) {
            Log.d(TAG, "GET request successful");
            Intent intent = new Intent(getApplicationContext(), ArriveActivity.class);
            intent.setFlags(Intent.FLAG_ACTIVITY_NEW_TASK);
            startActivity(intent);

            stopSelf();
        } else {
            Log.d(TAG, "GET request failed");
        }
    }
}
