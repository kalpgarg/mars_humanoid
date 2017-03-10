package kalp.dev.humanoidcontrol;

import android.app.Activity;
import android.content.Intent;
import android.os.Bundle;
import android.widget.Toast;

/**
 * Created by kalp garg on 25-02-2017.
 */

public class pidTuningActivity extends Activity {

    private String address=null;
    public static String DEVICE_ADDRESS="device_address";


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_pid_control);

        Intent intent=getIntent();
        address=intent.getStringExtra(MainActivity.DEVICE_ADDRESS);
        //Toast.makeText(this,address, Toast.LENGTH_SHORT).show();


    }
}
