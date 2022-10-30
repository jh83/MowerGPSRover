//Your WiFi credentials
const char ssid[] = "<SSID>";
const char password[] =  "<Password>";

//RTK2Go works well and is free
const char casterHost[] = "rtk2go.com"; 
const uint16_t casterPort = 2101;
const char casterUser[] = "<Email>"; //User must provide their own email address to use RTK2Go
const char casterUserPW[] = "";
const char mountPoint[] = "<MountPoint>"; //The mount point you want to get data from

//Emlid Caster also works well and is free
//const char casterHost[] = "caster.emlid.com";
//const uint16_t casterPort = 2101;
//const char casterUser[] = "u99696"; //User name and pw must be obtained through their web portal
//const char casterUserPW[] = "466zez";
//const char mountPoint[] = "MP1979"; //The mount point you want to get data from

// MongoDB Https endpoint and API-KEY
String serverName = "<URL to MongoDB HTTPS Endpoint>";
String apiKey = "<API_KEY>";
