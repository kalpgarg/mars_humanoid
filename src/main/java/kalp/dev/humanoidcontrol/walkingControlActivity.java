package kalp.dev.humanoidcontrol;

import android.app.Activity;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.content.Intent;
import android.os.AsyncTask;
import android.os.Bundle;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.Toast;

import java.io.IOException;
import java.util.UUID;

/**
 * Created by kalp garg on 25-02-2017.
 */

public class walkingControlActivity extends Activity {

    private String address=null;
    public static String DEVICE_ADDRESS="device_address";

    private EditText upDownSetpointEditText;
    private EditText upperASetpointEditText;
    private EditText lowerASetpointEditText;
    private EditText upperBSetpointEditText;
    private EditText lowerBSetpointEditText;
    private EditText rightLeftSetpointEditText;

    private Button upDownButton;
    private Button walkButton;
    private Button rightButton;
    private Button leftButton;

    private BluetoothAdapter myBluetooth=null;
    private BluetoothSocket btSocket=null;
    private boolean isBtConnected=false;
    static final UUID myUUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_walking_control);

        Intent intent=getIntent();
        address=intent.getStringExtra(MainActivity.DEVICE_ADDRESS);

        new ConnectBT().execute();

        upDownSetpointEditText=(EditText) findViewById(R.id.upDownSetpoint);
        upperASetpointEditText=(EditText) findViewById(R.id.upperASetpoint);
        lowerASetpointEditText=(EditText) findViewById(R.id.lowerASetpoint);
        upperBSetpointEditText=(EditText) findViewById(R.id.upperBSetpoint);
        lowerBSetpointEditText=(EditText) findViewById(R.id.lowerBSetpoint);
        rightLeftSetpointEditText=(EditText) findViewById(R.id.rightLeftSetpoint);

        upDownButton=(Button) findViewById(R.id.upDownButton);
        walkButton=(Button) findViewById(R.id.walkButton);
        rightButton=(Button) findViewById(R.id.rightButton);
        leftButton=(Button) findViewById(R.id.leftButton);

        upDownButton.setOnClickListener(upDownButtonOnClickListener);
        walkButton.setOnClickListener(walkButtonOnClickListener);
        rightButton.setOnClickListener(rightButtonOnClickListener);
        leftButton.setOnClickListener(leftButtonOnClickListener);
    }

    View.OnClickListener upDownButtonOnClickListener=new View.OnClickListener() {
        @Override
        public void onClick(View view) {
            if(btSocket != null){
                try{
                    String upDownSetpoint=upDownSetpointEditText.getText().toString();

                    btSocket.getOutputStream().write("u".getBytes());
                    btSocket.getOutputStream().write(upDownSetpoint.getBytes());
                }
                catch (IOException e){
                    Toast.makeText(walkingControlActivity.this, "Error in Sending Data", Toast.LENGTH_SHORT).show();
                }
            }
        }
    };

    View.OnClickListener walkButtonOnClickListener=new View.OnClickListener() {
        @Override
        public void onClick(View view) {
            if(btSocket != null){
                try{
                    String upperASetpoint=upperASetpointEditText.getText().toString()+",";
                    String lowerASetpoint=lowerASetpointEditText.getText().toString()+",";
                    String upperBSetpoint=upperBSetpointEditText.getText().toString()+",";
                    String lowerBSetpoint=lowerBSetpointEditText.getText().toString();

                    btSocket.getOutputStream().write("w".getBytes());
                    btSocket.getOutputStream().write(upperASetpoint.getBytes());
                    btSocket.getOutputStream().write(lowerASetpoint.getBytes());
                    btSocket.getOutputStream().write(upperBSetpoint.getBytes());
                    btSocket.getOutputStream().write(lowerBSetpoint.getBytes());
                }
                catch (IOException e){
                    Toast.makeText(walkingControlActivity.this, "Error in Sending Data", Toast.LENGTH_SHORT).show();
                }
            }

        }
    };

    View.OnClickListener rightButtonOnClickListener=new View.OnClickListener() {
        @Override
        public void onClick(View view) {
            if(btSocket != null){
                try{
                    String rightSetpoint=rightLeftSetpointEditText.getText().toString();

                    btSocket.getOutputStream().write("r".getBytes());
                    btSocket.getOutputStream().write(rightSetpoint.getBytes());
                }
                catch (IOException e){
                    Toast.makeText(walkingControlActivity.this, "Error in Sending Data", Toast.LENGTH_SHORT).show();
                }
            }
        }
    };

    View.OnClickListener leftButtonOnClickListener=new View.OnClickListener() {
        @Override
        public void onClick(View view) {
            if(btSocket != null){
                try{
                    String leftSetpoint=rightLeftSetpointEditText.getText().toString();

                    btSocket.getOutputStream().write("l".getBytes());
                    btSocket.getOutputStream().write(leftSetpoint.getBytes());
                }
                catch (IOException e){
                    Toast.makeText(walkingControlActivity.this, "Error in Sending Data", Toast.LENGTH_SHORT).show();
                }
            }
        }
    };

    private class ConnectBT extends AsyncTask<Void,Void,Void> {

        private boolean ConnectSuccess=true;
        @Override
        protected void onPreExecute() {
            super.onPreExecute();
            Toast.makeText(walkingControlActivity.this, "Connnecting to device "+ address, Toast.LENGTH_SHORT).show();
        }

        @Override
        protected Void doInBackground(Void... voids) {
            try
            {
                if (btSocket == null || !isBtConnected)
                {
                    myBluetooth = BluetoothAdapter.getDefaultAdapter();//get the mobile bluetooth device
                    BluetoothDevice dispositivo = myBluetooth.getRemoteDevice(address);//connects to the device's address and checks if it's available
                    btSocket = dispositivo.createInsecureRfcommSocketToServiceRecord(myUUID);//create a RFCOMM (SPP) connection
                    BluetoothAdapter.getDefaultAdapter().cancelDiscovery();
                    btSocket.connect();//start connection
                }
            }
            catch (IOException e)
            {
                ConnectSuccess = false;//if the try failed, you can check the exception here
            }
            return null;
        }

        @Override
        protected void onPostExecute(Void aVoid) {
            super.onPostExecute(aVoid);
            if (!ConnectSuccess)
            {
                Toast.makeText(walkingControlActivity.this, "Connection Failed. Try again.", Toast.LENGTH_SHORT).show();
                finish();
            }
            else
            {
                Toast.makeText(walkingControlActivity.this, "Connected", Toast.LENGTH_SHORT).show();
                isBtConnected = true;
            }
        }

    }




}
