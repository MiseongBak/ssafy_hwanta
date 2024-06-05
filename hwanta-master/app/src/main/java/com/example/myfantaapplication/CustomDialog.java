package com.example.myfantaapplication;

import android.app.Dialog;
import android.content.Context;
import android.content.Intent;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;

import androidx.annotation.NonNull;

public class CustomDialog extends Dialog {
    private TextView txt_contents;
    private Button shutdownClick;
    private Button callClick;
    private Context mContext;

    public CustomDialog(@NonNull Context context, String contents) {
        super(context);
        mContext = context;
        setContentView(R.layout.activity_custom_dialog);

        txt_contents = findViewById(R.id.txt_contents);
        txt_contents.setText(contents);
        shutdownClick = findViewById(R.id.btn_shutdown);
        shutdownClick.setOnClickListener(new View.OnClickListener() {

            @Override
            public void onClick(View v) {
                dismiss();
            }
        });

        callClick = findViewById(R.id.btn_call);
        callClick.setOnClickListener(new View.OnClickListener() {

            @Override
            public void onClick(View v) {
                Intent intent=new Intent(mContext, CallActivity.class);
                mContext.startActivity(intent);
                dismiss(); // 다이얼로그 닫기
            }
        });

    }
}
