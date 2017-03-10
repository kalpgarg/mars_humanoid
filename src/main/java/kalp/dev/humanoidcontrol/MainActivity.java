package kalp.dev.humanoidcontrol;

import android.app.Activity;
import android.app.ListActivity;
import android.content.Intent;
import android.content.res.Resources;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.view.Menu;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.view.View;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.ListView;
import android.widget.Toast;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.concurrent.ArrayBlockingQueue;

public class MainActivity extends AppCompatActivity {

    private ListView controlTypeListView;
    private ArrayAdapter<String> listAdapter;
    private String address=null;
    public static String DEVICE_ADDRESS="device_address";


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        controlTypeListView=(ListView) findViewById(R.id.mainListView);
        controlTypeListView.setOnItemClickListener(viewControlClickListener);

        Resources res=getResources();
        String[] controls=res.getStringArray(R.array.controls_array);

        ArrayList<String> controlsList=new ArrayList<>();
        controlsList.addAll(Arrays.asList(controls));

        listAdapter=new ArrayAdapter<String>(MainActivity.this,R.layout.control_type_text_view,controlsList);
        controlTypeListView.setAdapter(listAdapter);


    }

    @Override
    protected void onResume() {
        super.onResume();
        Intent getAddr=getIntent();
        address=getAddr.getStringExtra(AvailableDevices.DEVICE_ADDRESS);

    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        super.onCreateOptionsMenu(menu);
        MenuInflater menuInflater=getMenuInflater();
        menuInflater.inflate(R.menu.connect_new_bluetooth_device_menu,menu);
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        Intent connectDevice=new Intent(MainActivity.this,AvailableDevices.class);
        startActivity(connectDevice);
        return super.onOptionsItemSelected(item);
    }

    AdapterView.OnItemClickListener viewControlClickListener=new AdapterView.OnItemClickListener() {
        @Override
        public void onItemClick(AdapterView<?> adapterView, View view, int i, long l) {

            switch((int)l){
                case 0:
                    Intent pidIntent =new Intent(MainActivity.this,pidTuningActivity.class);
                    pidIntent.putExtra(DEVICE_ADDRESS,address);
                    startActivity(pidIntent);
                    break;
                case 1:
                    Intent motorIntent =new Intent(MainActivity.this,motorControlActivity.class);
                    motorIntent.putExtra(DEVICE_ADDRESS,address);
                    startActivity(motorIntent);
                    break;
                case 2:
                    Intent walkIntent =new Intent(MainActivity.this,walkingControlActivity.class);
                    walkIntent.putExtra(DEVICE_ADDRESS,address);
                    startActivity(walkIntent);
                    break;


            }
        }
    };

}
