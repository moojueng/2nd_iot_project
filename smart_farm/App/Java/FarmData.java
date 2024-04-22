package com.example.test2;


import android.content.Context;
import android.os.Bundle;

import androidx.appcompat.app.AppCompatActivity;

public class FarmData extends AppCompatActivity {
    private String member_cds;
    private String member_soil;
    private String member_hum;
    private String member_tem;
    private String member_war;
    private String member_date;
    public static Context context_main;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        context_main = this;

    }
    public String getMember_cds() {
        return member_cds;
    }

    public String getMember_soil() {
        return member_soil;
    }

    public String getMember_hum() {
        return member_hum;
    }
    public String getMember_tem() {
        return member_tem;
    }

    public String getMember_war() {
        return member_war;
    }

    public String getMember_date() {
        return member_date;
    }

    public void setMember_cds(String member_cds) {
        this.member_cds = member_cds;
    }

    public void setMember_soil(String member_soil) {
        this.member_soil = member_soil;
    }

    public void setMember_hum(String member_hum) {
        this.member_hum = member_hum;
    }
    public void setMember_tem(String member_tem) {
        this.member_tem = member_tem;
    }

    public void setMember_war(String member_war) {
        this.member_war = member_war;
    }

    public void setMember_date(String member_date) {
        this.member_date = member_date;
    }

}
