/**
 * Add gobals here
 */
var tab				= 1;
var seconds 		= null;
var otaTimerVar 	= null;

/**
 * Initialize functions here.
 */
$(document).ready(function(){
	getConnectionsConfiguration();
	getUpdateStatus();
	changeTab(tab);
	
	//setTimeout(setInterval(getConnectionsConfiguration, 10000));
});

function getConnectionsConfiguration()
{
    var xhr = new XMLHttpRequest();
    var requestURL = "/getConfiguration";		//ConnectionsConfiguration has to be handled
    xhr.open('POST', requestURL, false);
    xhr.send();
    
     if (xhr.readyState == 4 && xhr.status == 200) {
     	var response = JSON.parse(xhr.responseText);
     	
		document.getElementById("APP_EUI").value = response.APP_EUI;
		document.getElementById("DEV_EUI").value = response.DEV_EUI;
		document.getElementById("APP_KEY").value = response.APP_KEY;
		if (response.LORAWAN_STATUS == 1) document.getElementById("LORAWAN_STATUS").style.backgroundColor = 'green';
		else if (response.LORAWAN_STATUS == 0) document.getElementById("LORAWAN_STATUS").style.backgroundColor = 'orange';
		else document.getElementById("LORAWAN_STATUS").style.backgroundColor = 'red';
		
		document.getElementById("SEND_INTERVAL").value = response.SEND_INTERVAL;
		document.getElementById("STATION_1").checked = response.STATION_1;
		document.getElementById("STATION_2").checked = response.STATION_2;
		document.getElementById("STATION_3").checked = response.STATION_3;
		document.getElementById("STATION_4").checked = response.STATION_4;
		document.getElementById("STATION_5").checked = response.STATION_5;
		document.getElementById("STATION_6").checked = response.STATION_6;
		document.getElementById("STATION_7").checked = response.STATION_7;
		document.getElementById("STATION_8").checked = response.STATION_8;
		document.getElementById("DAVIS_DEBUG_TRACE").checked = response.DAVIS_DEBUG_TRACE;
		if (response.DAVIS_STATUS == 1) document.getElementById("DAVIS_STATUS").style.backgroundColor = 'green';
		else if (response.DAVIS_STATUS == 0) document.getElementById("DAVIS_STATUS").style.backgroundColor = 'orange';
		else document.getElementById("DAVIS_STATUS").style.backgroundColor = 'red';		
		
	}
}
   

/**
 * Gets file name and size for display on the web page.
 */        
function getFileInfo() 
{
    var x = document.getElementById("selected_file");
    var file = x.files[0];

    document.getElementById("file_info").innerHTML = "<h4>File: " + file.name + "<br>" + "Size: " + file.size + " bytes</h4>";
}

/**
 * Handles the firmware update.
 */
function updateFirmware() 
{
    // Form Data
    var formData = new FormData();
    var fileSelect = document.getElementById("selected_file");
    
    if (fileSelect.files && fileSelect.files.length == 1) 
	{
        var file = fileSelect.files[0];
        formData.set("file", file, file.name);
        document.getElementById("ota_update_status").innerHTML = "Uploading " + file.name + ", Firmware Update in Progress...";

        // Http Request
        var request = new XMLHttpRequest();

        request.upload.addEventListener("progress", updateProgress);
        request.open('POST', "/OTAupdate");		// OTAupdate has to be handled
        request.responseType = "blob";
        request.send(formData);
    } 
	else 
	{
        window.alert('Select A File First')
    }
}

/**
 * Progress on transfers from the server to the client (downloads).
 */
function updateProgress(oEvent) 
{
    if (oEvent.lengthComputable) 
	{
        getUpdateStatus();
    } 
	else 
	{
        window.alert('total size is unknown')
    }
}

/**
 * Posts the firmware udpate status.
 */
function getUpdateStatus() 
{
    var xhr = new XMLHttpRequest();
    var requestURL = "/OTAstatus";		//OTAstatus has to be handled
    xhr.open('POST', requestURL, false);
    xhr.send('ota_update_status');

    if (xhr.readyState == 4 && xhr.status == 200) 
	{		
        var response = JSON.parse(xhr.responseText);
						
	 	document.getElementById("latest_firmware").innerHTML = response.compile_date + " - " + response.compile_time

		// If flashing was complete it will return a 1, else -1
		// A return of 0 is just for information on the Latest Firmware request
        if (response.ota_update_status == 1) 
		{
    		// Set the countdown timer time
            seconds = 10;
            // Start the countdown timer
            otaRebootTimer();
        } 
        else if (response.ota_update_status == -1)
		{
            document.getElementById("ota_update_status").innerHTML = "!!! Upload Error !!!";
        }
    }
}

/**
 * Displays the reboot countdown.
 */
function otaRebootTimer() 
{	
    document.getElementById("ota_update_status").innerHTML = "OTA Firmware Update Complete. This page will close shortly, Rebooting in: " + seconds;

    if (--seconds == 0) 
	{
        clearTimeout(otaTimerVar);
        window.location.reload();
    } 
	else 
	{
        otaTimerVar = setTimeout(otaRebootTimer, 1000);
    }
}

/**
 * Clear HTML div
 */
function clearDiv(divID){
	document.getElementById(divID).innerHTML = "";
}

/**
 * Remove HTML div 
 */
function removeDiv(divID){
	document.getElementById(divID).remove();
}

function submitLoRaWANConf()
{
	var xhr = new XMLHttpRequest();
    var requestURL = "/setLoRaWANConfiguration";		//setLoRaWANConfiguration has to be handled
    xhr.open('POST', requestURL, false);
    xhr.send(document.getElementById('APP_EUI').value + "\n" 
			+ document.getElementById('DEV_EUI').value + "\n"
    		+ document.getElementById('APP_KEY').value + "\0");
    
    if (xhr.readyState == 4 && xhr.status == 200) {
    	window.alert(xhr.responseText);
    }else window.alert("Something went wrong!");
    
	document.getElementById('APP_EUI').disabled = true;
	document.getElementById('DEV_EUI').disabled = true;
	document.getElementById('APP_KEY').disabled = true;
	document.getElementById('LORAWAN_EDIT').hidden = false;
	document.getElementById('LORAWAN_SUBMIT').hidden = true;
	getConnectionsConfiguration();
}

function editLoRaWANConf()
{
	document.getElementById('APP_EUI').disabled = false;
	document.getElementById('DEV_EUI').disabled = false;
	document.getElementById('APP_KEY').disabled = false;
	document.getElementById('LORAWAN_EDIT').hidden = true;
	document.getElementById('LORAWAN_SUBMIT').hidden = false;
	getConnectionsConfiguration();
}

function submitDAVISConf()
{
	//window.alert("NTP Server: " + document.getElementById('NTP_SERVER').value + "\n" + "Sync time (secs): " + document.getElementById('NTP_SYNC').value + "\n");
	
    var xhr = new XMLHttpRequest();
    var requestURL = "/setDAVISConfiguration";		//setNTPConfiguration has to be handled
    xhr.open('POST', requestURL, false);
    xhr.send(document.getElementById('SEND_INTERVAL').value + "\n" 
			+ document.getElementById('STATION_1').checked + "\n"
			+ document.getElementById('STATION_2').checked + "\n"
			+ document.getElementById('STATION_3').checked + "\n"
			+ document.getElementById('STATION_4').checked + "\n"
			+ document.getElementById('STATION_5').checked + "\n"
			+ document.getElementById('STATION_6').checked + "\n"
			+ document.getElementById('STATION_7').checked + "\n"
			+ document.getElementById('STATION_8').checked + "\n"
			+ document.getElementById('DAVIS_DEBUG_TRACE').checked + "\0");
    
    if (xhr.readyState == 4 && xhr.status == 200) {
    	window.alert(xhr.responseText);
    }
    
	document.getElementById('SEND_INTERVAL').disabled = true;
	document.getElementById("STATION_1").disabled = true;
	document.getElementById("STATION_2").disabled = true;
	document.getElementById("STATION_3").disabled = true;
	document.getElementById("STATION_4").disabled = true;
	document.getElementById("STATION_5").disabled = true;
	document.getElementById("STATION_6").disabled = true;
	document.getElementById("STATION_7").disabled = true;
	document.getElementById("STATION_8").disabled = true;
	document.getElementById("DAVIS_DEBUG_TRACE").disabled = true;
	document.getElementById('DAVIS_EDIT').hidden = false;
	document.getElementById('DAVIS_SUBMIT').hidden = true;
	getConnectionsConfiguration();
}

function editDAVISConf()
{
	document.getElementById('SEND_INTERVAL').disabled = false;
	document.getElementById("STATION_1").disabled = false;
	document.getElementById("STATION_2").disabled = false;
	document.getElementById("STATION_3").disabled = false;
	document.getElementById("STATION_4").disabled = false;
	document.getElementById("STATION_5").disabled = false;
	document.getElementById("STATION_6").disabled = false;
	document.getElementById("STATION_7").disabled = false;
	document.getElementById("STATION_8").disabled = false;
	document.getElementById("DAVIS_DEBUG_TRACE").disabled = false;
	document.getElementById('DAVIS_EDIT').hidden = true;
	document.getElementById('DAVIS_SUBMIT').hidden = false;
	getConnectionsConfiguration();
}

function restartEsp(){
	
	// Http Request
	var request = new XMLHttpRequest();
	var requestURL = "/RestartESP";		//RestartESP has to be handled
	
	request.open('POST', requestURL);
	
	request.send();
	
	window.location.reload();
}

function ClearAndRestartEsp(){
	
	// Http Request
	var request = new XMLHttpRequest();
	var requestURL = "/ClearAndRestartESP";		//ClearAndRestartESP has to be handled
	
	request.open('POST', requestURL);
	
	request.send();
}

function changeTab(number){
	
	unlockTab();
			
	if(number == 1){
		$('#configurations_button').attr('disabled', true);
		document.getElementById('CONFIGURATIONS').style.display = "block";
	}
	else if(number == 2){
		$('#ota_button').attr('disabled', true);
		document.getElementById('OTA').style.display = "block";
	}
	
	tab = number;
}

function unlockTab(){
	
	if(tab == 1){
		$('#configurations_button').attr('disabled', false);
		document.getElementById('CONFIGURATIONS').style.display = "none";
	}
	else if(tab == 2){
		$('#ota_button').attr('disabled', false);
		document.getElementById('OTA').style.display = "none";
	}
}

function lockEdits(edit_type){
	$('input.'+ edit_type).attr('disabled', true);
}

function unlockEdits(edit_type){
	$('input.'+ edit_type).attr('disabled', false);
}
