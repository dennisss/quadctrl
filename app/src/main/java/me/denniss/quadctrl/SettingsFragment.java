package me.denniss.quadctrl;

import android.app.ProgressDialog;
import android.os.AsyncTask;
import android.os.Bundle;
import android.app.Fragment;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.SeekBar;


public class SettingsFragment extends Fragment {

    public SettingsFragment() {
        // Required empty public constructor
    }

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,
                             Bundle savedInstanceState) {
        // Inflate the layout for this fragment
        return inflater.inflate(R.layout.fragment_settings, container, false);
    }

    @Override
    public void onViewCreated(View view, Bundle savedInstanceState) {
        super.onViewCreated(view, savedInstanceState);

        /*
        int[] ids = new int[]{ R.id.seekBar, R.id.seekBar2, R.id.seekBar3, R.id.seekBar4 };

        final SeekBar[] bars = new SeekBar[ids.length];
        for(int i = 0; i < ids.length; i++){
            bars[i] = (SeekBar) view.findViewById(ids[i]);
            bars[i].setMax(100);
        }



        Button speedBtn = (Button) view.findViewById(R.id.speedBtn);
        speedBtn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {

                float[] speeds = new float[4];

                for(int i = 0; i < 4; i++){
                    speeds[i] = ((float)bars[i].getProgress()) / 100.0f;
                }

                Quadcopter.setMotors(speeds);
            }
        });

        Button zeroBtn = (Button) view.findViewById(R.id.zeroBtn);
        zeroBtn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                float[] speeds = new float[]{0,0,0,0};

                for(int i = 0; i < 4; i++){
                    bars[i].setProgress(0);
                }

                Quadcopter.setMotors(speeds);
            }
        });
        */

        Button startBtn = (Button) view.findViewById(R.id.startBtn);
        startBtn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                ControlNode.start();
            }
        });

        Button stopBtn = (Button) view.findViewById(R.id.stopBtn);
        stopBtn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                ControlNode.stop();
            }
        });


        Button calibBtn = (Button) view.findViewById(R.id.calibBtn);
        calibBtn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                final ProgressDialog dialog = ProgressDialog.show(SettingsFragment.this.getActivity(), "Calibrating", "Please keep your device still", true);

                new AsyncTask<Void, Void, Void>() {

                    protected Void doInBackground(Void... arg0) {
                        ControlNode.calibrate();
                        return null;
                    }

                    protected void onPostExecute(Void result) {
                        dialog.dismiss();
                    }
                }.execute();

            }
        });







    }
}
