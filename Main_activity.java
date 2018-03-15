/**
 * Rescue Robotics 2016 App
 * Developed by Cognitive Anteater Robotics Laboratory at University of California, Irvine
 * Controls wheeled robot through IOIO
 * Parts of code adapted from OpenCV blob follow
 * Before running, connect phone to IOIO with a bluetooth connection
 * If you would like to uncomment sections for message passing, first connect peer phones using wifi direct
 */
package abr.main;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.Calendar;
import java.util.List;

import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewListener2;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import com.google.android.gms.common.ConnectionResult;
import com.google.android.gms.common.api.GoogleApiClient;
import com.google.android.gms.common.api.GoogleApiClient.ConnectionCallbacks;
import com.google.android.gms.common.api.GoogleApiClient.OnConnectionFailedListener;
import com.google.android.gms.location.LocationListener;
import com.google.android.gms.location.LocationRequest;
import com.google.android.gms.location.LocationServices;
import com.google.zxing.BinaryBitmap;
import com.google.zxing.ChecksumException;
import com.google.zxing.FormatException;
import com.google.zxing.LuminanceSource;
import com.google.zxing.NotFoundException;
import com.google.zxing.RGBLuminanceSource;
import com.google.zxing.Reader;
import com.google.zxing.Result;
import com.google.zxing.common.HybridBinarizer;
import com.google.zxing.qrcode.QRCodeReader;

import ioio.lib.util.IOIOLooper;
import ioio.lib.util.IOIOLooperProvider;
import ioio.lib.util.android.IOIOAndroidApplicationHelper;
import android.graphics.Bitmap;
import android.graphics.Color;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.location.Location;
import android.media.AudioManager;
import android.media.ToneGenerator;
import android.os.Bundle;
import android.os.Environment;
import android.app.Activity;
import android.content.Context;
import android.content.Intent;
import android.util.Log;
import android.view.View;
import android.view.Window;
import android.view.WindowManager;
import android.view.View.OnClickListener;
import android.widget.Button;
import android.widget.TextView;
import android.widget.ToggleButton;

public class Main_activity extends Activity implements IOIOLooperProvider, SensorEventListener, ConnectionCallbacks, OnConnectionFailedListener,
		CvCameraViewListener2 // implements IOIOLooperProvider: from IOIOActivity
{
	private final IOIOAndroidApplicationHelper helper_ = new IOIOAndroidApplicationHelper(this, this); // from IOIOActivity

	boolean isSame;
	boolean redRobot = true; //account for different gear ratios
	int forwardSpeed;
	int turningSpeed;
	int obstacleTurningSpeed;

	// ioio variables
	IOIO_thread m_ioio_thread;

	//blob detection variables
	private CameraBridgeViewBase mOpenCvCameraView;
	private Mat mRgba;
	private Scalar mBlobColorRgba;
	private ColorBlobDetector mDetector, mDetector2;
	private Mat mSpectrum;
	private Scalar CONTOUR_COLOR;

	//app state variables
	private boolean autoMode;

	//variables for logging
	private Sensor mGyroscope;
	private Sensor mGravityS;
	float[] mGravity;
	float[] mGyro;
	int countobj=0;

	//location variables
	private GoogleApiClient mGoogleApiClient;
	private double curr_lat;
	private double curr_lon;
	private Location curr_loc;
	private LocationRequest mLocationRequest;
	private LocationListener mLocationListener;
	Location dest_loc;
	float distance = 0;

	//variables for compass
	private SensorManager mSensorManager;
	private Sensor mCompass, mAccelerometer;
	float[] mAcc;
	float[] mGeomagnetic;
	public float heading = 0;
	public float bearing;
	int areaCount = 0;
	int areaCount2 = 0;
	int colorcounter = 150;
	int colorflag = 1;

	//sockets for message passing
	Boolean isClient = true;
	ServerSocket serverSocket;
	Socket socket;
	Socket clientSocket;
	DataInputStream dataInputStream;
	DataOutputStream dataOutputStream;


	//timers
	Long startTime;
	Long currTime;

	//pan/tilt
	int panVal=1500;
	int tiltVal=1500;
	boolean panningRight = false;
	boolean tiltingUp = false;
	int panInc;
	int tiltInc;

	boolean stationary = true;
	boolean configured=false;

	// called to use OpenCV libraries contained within the app as opposed to a separate download
	static {
		if (!OpenCVLoader.initDebug()) {
			// Handle initialization error
		}
	}

	// called whenever the activity is created
	public void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);

		requestWindowFeature(Window.FEATURE_NO_TITLE);
		getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
		setContentView(R.layout.main);

		helper_.create(); // from IOIOActivity

		//set up opencv camera
		mOpenCvCameraView = (CameraBridgeViewBase) findViewById(R.id.color_blob_detection_activity_surface_view);
		mOpenCvCameraView.setCvCameraViewListener(this);
		mOpenCvCameraView.enableView();

		//initialize textviews

		//add functionality to autoMode button
	/*	Button buttonAuto = (Button) findViewById(R.id.btnAuto);
		buttonAuto.setOnClickListener(new OnClickListener() {
			public void onClick(View v) {
				if (!autoMode) {
					v.setBackgroundResource(R.drawable.button_auto_on);
					autoMode = true;
					startTime = System.currentTimeMillis();
				} else {
					v.setBackgroundResource(R.drawable.button_auto_off);
					autoMode = false;
				}
			}
		});

		//set starting autoMode button color
		if (autoMode) {
			buttonAuto.setBackgroundResource(R.drawable.button_auto_on);
		} else {
			buttonAuto.setBackgroundResource(R.drawable.button_auto_off);
		}

*/


		//set up compass
		mSensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
	    mCompass= mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
	    mAccelerometer= mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
	    mGyroscope = mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
	    mGravityS = mSensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY);


	}
	//Method necessary for google play location services
	protected synchronized void buildGoogleApiClient() {
	    mGoogleApiClient = new GoogleApiClient.Builder(this)
	        .addConnectionCallbacks(this)
	        .addOnConnectionFailedListener(this)
	        .addApi(LocationServices.API)
	        .build();
	}
	//Method necessary for google play location services
	@Override
    public void onConnected(Bundle connectionHint) {
        // Connected to Google Play services
		curr_loc = LocationServices.FusedLocationApi.getLastLocation(mGoogleApiClient);
	    startLocationUpdates();
    }
	//Method necessary for google play location services
	protected void startLocationUpdates() {
	    LocationServices.FusedLocationApi.requestLocationUpdates(mGoogleApiClient, mLocationRequest, mLocationListener);
	}
	//Method necessary for google play location services
    @Override
    public void onConnectionSuspended(int cause) {
        // The connection has been interrupted.
        // Disable any UI components that depend on Google APIs
        // until onConnected() is called.
    }
    //Method necessary for google play location services
    @Override
    public void onConnectionFailed(ConnectionResult result) {
        // This callback is important for handling errors that
        // may occur while attempting to connect with Google.
        //
        // More about this in the 'Handle Connection Failures' section.
    }
	@Override
	public final void onAccuracyChanged(Sensor sensor, int accuracy) {
		// Do something here if sensor accuracy changes.
	}

	// Called whenever the value of a sensor changes
	// Should be called whenever a sensor changes.
	// Good time to get IR values and send move commands to the robot

	@Override
	public final void onSensorChanged(SensorEvent event) {
		if(m_ioio_thread != null) {
			//	setText(String.format("%.3f", m_ioio_thread.getIrCenterReading()), irCenterText);
		}

	}
	//boolean moving = false;
	public void onButtonClick(View v){
		if(v.getId() == R.id.up_button) {
			m_ioio_thread.set_left(1700);
			m_ioio_thread.set_right(1720);
			//move(1720);
			//moving = true;

			System.out.println ("MOVING FORWARD!");
		}
		else if(v.getId() == R.id.down_button) {
			m_ioio_thread.stay();
			//m_ioio_thread.set_left(1200);
			//m_ioio_thread.set_right(1200);
			//move(1200);
			System.out.println ("MOVING BACKWARD!");
		}
		else if(v.getId() == R.id.turn_left_button) {
			m_ioio_thread.turn_left(1600);
			System.out.println ("MOVING LEFT!");
		}
		else if(v.getId() == R.id.turn_right_button) {
			m_ioio_thread.turn_right(1600);
			System.out.println ("MOVING RIGHT!");
		}
		else if(v.getId() == R.id.start_stop_button){
			//m_ioio_thread.set_left(1500);
			//m_ioio_thread.set_right(1500);
			m_ioio_thread.stay();
			configured=true;
			System.out.println ("MOVING NOT!");
		}
		else{}
	}

	/*private void stay(){
		m_ioio_thread.set_left(1500);
		m_ioio_thread.set_right(1500);
	}
	private void move(int value){
		m_ioio_thread.set_left(value);
		m_ioio_thread.set_right(value);
	}
	private void turn_left(int value){
		m_ioio_thread.set_left(value);
		m_ioio_thread.set_right(value+200);
	}
	private void turn_right(int value){
		m_ioio_thread.set_left(value+200);
		m_ioio_thread.set_right(value);
	}*/
	private ToggleButton btnStartStop;
	private void enableUi(final boolean enable) {
		runOnUiThread(new Runnable() {
			@Override
			public void run() {
				btnStartStop.setEnabled(enable);
			}
		});
	}


	//Scan for QR code and save information to phone
	public String scan(Mat frame) {
		Bitmap bMap = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.ARGB_8888);
		Utils.matToBitmap(frame, bMap);
		int[] intArray = new int[bMap.getWidth()*bMap.getHeight()];
		//copy pixel data from the Bitmap into the 'intArray' array
		bMap.getPixels(intArray, 0, bMap.getWidth(), 0, 0, bMap.getWidth(), bMap.getHeight());

		LuminanceSource source = new RGBLuminanceSource(bMap.getWidth(), bMap.getHeight(),intArray);

		BinaryBitmap bitmap = new BinaryBitmap(new HybridBinarizer(source));
		Reader reader = new QRCodeReader();

		String text;

	    try {
			Result result = reader.decode(bitmap);
			text = result.getText();
			Calendar calendar = Calendar.getInstance();
			java.util.Date now = calendar.getTime();
			java.sql.Timestamp currentTimestamp = new java.sql.Timestamp(now.getTime());
			String time = currentTimestamp.toString();
			curr_lat = curr_loc.getLatitude();
			curr_lon = curr_loc.getLongitude();
			String info = text +" ,Lat:"+curr_lat+" ,Lon:"+curr_lon+" ,Time:"+time;
			try {
			    File newFolder = new File(Environment.getExternalStorageDirectory(), "RescueRobotics_Heat2");
				//File newFolder = new File(Environment.getDataDirectory(), "RescueRobotics_Heat1");
			    if (!newFolder.exists()) {
			        newFolder.mkdirs();
			    }
			    try {
			        File file = new File(newFolder, time + ".txt");
			        file.createNewFile();
			        FileOutputStream fos=new FileOutputStream(file);
	                try {
	                	byte[] b = info.getBytes();
	                    fos.write(b);
	                    fos.close();
	                   // ToneGenerator toneG = new ToneGenerator(AudioManager.STREAM_ALARM, 100);
	        		//	toneG.startTone(ToneGenerator.TONE_CDMA_PIP, 200);
	                } catch (IOException e) {
	                	Log.e("app.main","Couldn't write to SD");
	                }
			    } catch (Exception ex) {
			    	Log.e("app.main","Couldn't write to SD");
			    }
			} catch (Exception e) {
			    Log.e("app.main","Couldn't write to SD");
			}
			Log.i("rescue robotics",text);
			return text;
		} catch (NotFoundException e) {
			e.printStackTrace();
			text = "no code found";
		} catch (ChecksumException e) {
			e.printStackTrace();
			text =  "checksum error";
		} catch (FormatException e) {
			e.printStackTrace();
			text = "format error";
		}
	    Log.i("rescue robotics",text);

		return text;
	}



	//Called whenever activity resumes from pause
	@Override
	public void onResume() {
		super.onResume();
	    if (mOpenCvCameraView != null)
			mOpenCvCameraView.enableView();
	    mSensorManager.registerListener(this, mCompass, SensorManager.SENSOR_DELAY_NORMAL);
	    mSensorManager.registerListener(this, mAccelerometer, SensorManager.SENSOR_DELAY_NORMAL);
	    mSensorManager.registerListener(this, mGyroscope, SensorManager.SENSOR_DELAY_NORMAL);
	    mSensorManager.registerListener(this, mGravityS, SensorManager.SENSOR_DELAY_NORMAL);
	}

	//Called when activity pauses
	@Override
	public void onPause() {
		super.onPause();
		if (mOpenCvCameraView != null)
			mOpenCvCameraView.disableView();
		mSensorManager.unregisterListener(this);
//		stopLocationUpdates();
	}

	protected void stopLocationUpdates() {
	    LocationServices.FusedLocationApi.removeLocationUpdates(mGoogleApiClient, mLocationListener);
	}

	//Called when activity restarts. onCreate() will then be called
	@Override
	public void onRestart() {
		super.onRestart();
		Log.i("activity cycle","main activity restarting");
	}

	//Called when camera view starts. change bucket color here
	public void onCameraViewStarted(int width, int height) {
		mRgba = new Mat(height, width, CvType.CV_8UC4);
		mDetector = new ColorBlobDetector();
		mDetector2 = new ColorBlobDetector();
		mSpectrum = new Mat();
		mBlobColorRgba = new Scalar(255);
		CONTOUR_COLOR = new Scalar(255, 0, 0, 255);

		//To set color, find HSV values of desired color and convert each value to 1-255 scale
		mDetector.setHsvColor(new Scalar(228.0, 61.0, 77.0));//(240,129,136));//(217,90, 96));//(21,251,122));
		mDetector2.setHsvColor(new Scalar(5,5,5));//(181, 184, 181));//(5,5,5));//(250,251,251));//(237, 238, 238));//(200,53,42));


	}
	//Called when camera view stops
	public void onCameraViewStopped() {
		mRgba.release();
	}
	//Called at every camera frame. Main controls of the robot movements are in this function
	public Mat onCameraFrame(CvCameraViewFrame inputFrame) {
		Log.i("hahaha","area:"+((mDetector.getMaxArea())/(mDetector.getCenterX()*mDetector.getCenterY())) );

		mRgba = inputFrame.rgba();
		mDetector.process(mRgba);
		mDetector2.process(mRgba);

		List<MatOfPoint> contours = mDetector.getContours();
		List<MatOfPoint> contours2 = mDetector2.getContours();
		// Log.e("rescue robotics", "Contours count: " + contours.size());

		Imgproc.drawContours(mRgba, contours, -1, CONTOUR_COLOR);
		Imgproc.drawContours(mRgba, contours2, -1, CONTOUR_COLOR);

		Mat colorLabel = mRgba.submat(4, 68, 4, 68);
		colorLabel.setTo(mBlobColorRgba);

		Mat spectrumLabel = mRgba.submat(4, 4 + mSpectrum.rows(), 70,
				70 + mSpectrum.cols());
		mSpectrum.copyTo(spectrumLabel);



		//if(colorflag == 0) {
		System.out.println ("MOVING WHITE VALUEEEEEE WHITE: "+(( (mDetector2.getMaxArea())/(mDetector2.getCenterX()*mDetector2.getCenterY()*4))*100000));
		if ((((mDetector2.getMaxArea()) / (mDetector2.getCenterX() * mDetector2.getCenterY() * 4)) * 100000 > 3)) {
			areaCount2++;
			System.out.println("MOVING WHITE areaCount2: "+areaCount2);
			System.out.println("MOVING WHITE STATIONARY:"+stationary);
			if ((areaCount2 == 5 && stationary==true)) {
				areaCount=0;
				areaCount2=0;
				System.out.println("MOVING areaCount ---------------------------MOVING FOR WHITE PERSON FOUND---------------------------------------------");

				m_ioio_thread.move(1800);
				m_ioio_thread.move = 1;
				stationary=false;

			}

		}
		System.out.println ("MOVING STOP RED VALUEEEEEE RED: "+(( (mDetector.getMaxArea())/(mDetector.getCenterX()*mDetector.getCenterY()*4))*100000));
		if ( (((mDetector.getMaxArea()) / (mDetector.getCenterX() * mDetector.getCenterY() * 4)) * 100000 > 3)) {
			areaCount++;
			System.out.println("MOVING RED areaCount: "+areaCount);
			System.out.println("MOVING RED STATIONARY:"+stationary);
			if ((areaCount == 3 && stationary==false)) {
				areaCount=0;
				areaCount2=0;
				System.out.println("MOVING areaCount ---------------------------STOPPING FOR RED HAND------------------------------------------------");

				m_ioio_thread.stay();
				m_ioio_thread.move = 0;
				stationary=true;


			}
		}

		return mRgba;
	}


	//revert any degree measurement back to the -179 to 180 degree scale
	public float fixWraparound(float deg){
		if(deg <= 180.0 && deg > -179.99)
			return deg;
		else if(deg > 180)
			return deg-360;
		else
			return deg+360;

	}

	//determine whether 2 directions are roughly pointing in the same direction, correcting for angle wraparound
	public boolean sameDir(float dir1, float dir2){
		float dir = bearing%360;
		float headingMod = heading%360;
		//return (Math.abs((double) (headingMod - dir)) < 22.5 || Math.abs((double) (headingMod - dir)) > 337.5);
		return (Math.abs((double) (headingMod - dir)) < 2.5 || Math.abs((double) (headingMod - dir)) > 357.5);
	}

	//set the text of any text view in this application
	public void setText(final String str, final TextView tv)
	{
		  runOnUiThread(new Runnable() {
			  @Override
			  public void run() {
				  tv.setText(str);
			  }
		  });
	}




	/****************************************************** functions from IOIOActivity *********************************************************************************/

	/**
	 * Create the {@link IOIO_thread}. Called by the
	 * {@link IOIOAndroidApplicationHelper}. <br>
	 * Function copied from original IOIOActivity.
	 *
	 * */
	@Override
	public IOIOLooper createIOIOLooper(String connectionType, Object extra) {
		if (m_ioio_thread == null
				&& connectionType
						.matches("ioio.lib.android.bluetooth.BluetoothIOIOConnection")) {
			m_ioio_thread = new IOIO_thread();
			return m_ioio_thread;
		} else
			return null;
	}

	@Override
	protected void onDestroy() {
		super.onDestroy();
		Log.i("activity cycle","main activity being destroyed");
		helper_.destroy();
		if (mOpenCvCameraView != null)
			mOpenCvCameraView.disableView();
		if (mOpenCvCameraView != null)
			mOpenCvCameraView.disableView();
	}

	@Override
	protected void onStart() {
		super.onStart();
		Log.i("activity cycle","main activity starting");
		helper_.start();
	}

	@Override
	protected void onStop() {
		Log.i("activity cycle","main activity stopping");
		super.onStop();
		helper_.stop();
		mGoogleApiClient.disconnect();
		try {
			if(socket != null)
				socket.close();
			if(serverSocket != null)
				serverSocket.close();
			if(clientSocket != null)
				clientSocket.close();
		} catch (IOException e) {
			Log.e("rescue robotics", e.getMessage());
		}

	}

	@Override
	protected void onNewIntent(Intent intent) {
		super.onNewIntent(intent);
			if ((intent.getFlags() & Intent.FLAG_ACTIVITY_NEW_TASK) != 0) {
			helper_.restart();
		}
	}

}
