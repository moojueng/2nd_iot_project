package com.example.test2;

import android.app.Activity;
import androidx.annotation.NonNull;
import androidx.recyclerview.widget.RecyclerView;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.TextView;

import java.util.ArrayList;


public class DataAdapter extends RecyclerView.Adapter<DataAdapter.CustomViewHolder> {

    private ArrayList<FarmData> mList = null;
    private Activity context = null;


    public DataAdapter(Activity context, ArrayList<FarmData> list) {
        this.context = context;
        this.mList = list;
    }

    class CustomViewHolder extends RecyclerView.ViewHolder {
        protected TextView cds;
        protected TextView tem;
        protected TextView hum;
        protected TextView date;
        protected TextView soil;
        protected TextView war;


        public CustomViewHolder(View view) {
            super(view);
            this.date = (TextView) view.findViewById(R.id.textView_list_date);
            this.cds = (TextView) view.findViewById(R.id.textView_list_cds);
            this.hum = (TextView) view.findViewById(R.id.textView_list_hum);
            this.war = (TextView) view.findViewById(R.id.textView_list_war);
            this.soil = (TextView) view.findViewById(R.id.textView_list_soil);
            this.tem = (TextView) view.findViewById(R.id.textView_list_tem);
        }
    }


    @Override
    public CustomViewHolder onCreateViewHolder(ViewGroup viewGroup, int viewType) {
        View view = LayoutInflater.from(viewGroup.getContext()).inflate(R.layout.item_list, null);
        CustomViewHolder viewHolder = new CustomViewHolder(view);

        return viewHolder;
    }

    @Override
    public void onBindViewHolder(@NonNull CustomViewHolder viewholder, int position) {

        viewholder.cds.setText(mList.get(position).getMember_cds());
        viewholder.tem.setText(mList.get(position).getMember_tem());
        viewholder.hum.setText(mList.get(position).getMember_hum());
        viewholder.soil.setText(mList.get(position).getMember_soil());
        viewholder.war.setText(mList.get(position).getMember_war());
        viewholder.date.setText(mList.get(position).getMember_date());
    }

    @Override
    public int getItemCount() {
        return (null != mList ? mList.size() : 0);
    }

}
