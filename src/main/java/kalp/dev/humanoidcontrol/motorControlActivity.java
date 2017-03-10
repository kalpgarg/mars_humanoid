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

public class motorControlActivity extends Activity {

    private String address=null;
    public static String DEVICE_ADDRESS="device_address";

    private EditText editTextMotorAUpper1;
    private EditText editTextMotorAUpper2;
    private EditText editTextMotorALower1;
    private EditText editTextMotorALower2;
    private EditText editTextMotorBUpper1;
    private EditText editTextMotorBUpper2;
    private EditText editTextMotorBLower1;
    private EditText editTextMotorBLower2;

    private Button sendButton;

    private BluetoothAdapter myBluetooth=null;
    private BluetoothSocket btSocket=null;
    private boolean isBtConnected=false;
    static final UUID myUUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_motor_control);

        Intent intent=getIntent();
        address=intent.getStringExtra(MainActivity.DEVICE_ADDRESS);

        new ConnectBT().execute();

        editTextMotorAUpper1=(EditText)findViewById(R.id.motorAupper1speed);
        editTextMotorAUpper2=(EditText) findViewById(R.id.motorAupper2speed);
        editTextMotorALower1=(EditText) findViewById(R.id.motorAlower1speed);
        editTextMotorALower2=(EditText) findViewById(R.id.motorAlower2speed);
        editTextMotorBUpper1=(EditText) findViewById(R.id.motorBupper1speed);
        editTextMotorBUpper2=(EditText) findViewById(R.id.motorBupper2speed);
        editTextMotorBLower1=(EditText) findViewById(R.id.motorBlower1speed);
        editTextMotorBLower2=(EditText) findViewById(R.id.motorBlower2speed);

        sendButton=(Button) findViewById(R.id.motorControlSendButton);

        sendButton.setOnClickListener(sendButtonOnClickListener);

    }

    @Override
    protected void onStop() {
        super.onStop();
        try
        {
            btSocket.close(); //close connection
        }
        catch (IOException e)
        { }
    }

    View.OnClickListener sendButtonOnClickListener=new View.OnClickListener() {
        @Override
        public void onClick(View view) {
            if(btSocket!=null){
                try{
                    String motorAUpper1=editTextMotorAUpper1.getText().toString()+",";
                    String motorAUpper2=editTextMotorAUpper2.getText().toString()+",";
                    String motorALower1=editTextMotorALower1.getText().toString()+",";
                    String motorALower2=editTextMotorALower2.getText().toString()+",";
                    String motorBUpper1=editTextMotorBUpper1.getText().toString()+",";
                    String motorBUpper2=editTextMotorBUpper2.getText().toString()+",";
                    String motorBLower1=editTextMotorBLower1.getText().toString()+",";
                    String motorBLower2=editTextMotorBLower2.getText().toString();


                    btSocket.getOutputStream().write("m".getBytes());
                    btSocket.getOutputStream().write(motorAUpper1.getBytes());
                    btSocket.getOutputStream().write(motorAUpper2.getBytes());
                    btSocket.getOutputStream().write(motorALower1.getBytes());
                    btSocket.getOutputStream().write(motorALower2.getBytes());
                    btSocket.getOutputStream().write(motorBUpper1.getBytes());
                    btSocket.getOutputStream().write(motorBUpper2.getBytes());
                    btSocket.getOutputStream().write(motorBLower1.getBytes());
                    btSocket.getOutputStream().write(motorBLower2.getBytes());
                    btSocket.getOutputStream().write("c".getBytes());
                    Toast.makeText(motorControlActivity.this, "Data Send", Toast.LENGTH_SHORT).show();
                }
                catch(IOException e){
                    Toast.makeText(motorControlActivity.this, "Error in sending data", Toast.LENGTH_SHORT).show();
                }
            }
        }
    };

    private class ConnectBT extends AsyncTask<Void,Void,Void>{

        private boolean ConnectSuccess=true;
        @Override
        protected void onPreExecute() {
            super.onPreExecute();
            Toast.makeText(motorControlActivity.this, "Connnecting to device "+ address, Toast.LENGTH_SHORT).show();
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
                Toast.makeText(motorControlActivity.this, "Connection Failed. Try again.", Toast.LENGTH_SHORT).show();
                finish();
            }
            else
            {
                Toast.makeText(motorControlActivity.this, "Connected", Toast.LENGTH_SHORT).show();
                isBtConnected = true;
            }
        }

    }

}
