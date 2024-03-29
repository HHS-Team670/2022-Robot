'use strict';
Object.defineProperty(exports, '__esModule', { value: true });

const electron = require('electron');
const wpilib_NT = require('wpilib-nt-client');
const client = new wpilib_NT.Client();

client.startDebug("Debug");

// The client will try to reconnect after 1 second
//COMMENTED THIS OUT AND IT SEEMS TO WORK. TECHNICALLY SHOULDN'T BE COMMENTED OUT?
client.setReconnectDelay(25);

/** Module to control application life. */
const app = electron.app;

/** Module to create native browser window.*/
const BrowserWindow = electron.BrowserWindow;

/** Module for receiving messages from the BrowserWindow */
const ipc = electron.ipcMain;

// Keep a global reference of the window object, if you don't, the window will
// be closed automatically when the JavaScript object is garbage collected.
/**
 * The Main Window of the Program
 * @type {Electron.BrowserWindow}
 * */
let mainWindow;
var window = null;

let connectedFunc,
    ready = false;

let clientDataListener = (key, val, valType, mesgType, id, flags) => {
    if (val === 'true' || val === 'false') {
        val = val === 'true';
    }
    mainWindow.webContents.send(mesgType, {
        key,
        val,
        valType,
        id,
        flags
    });
};
function createWindow() {
    // Attempt to connect to the localhost
    client.start((con, err) => {

        let connectFunc = () => {
            console.log('---Sending status');
            mainWindow.webContents.send('connected', con);

            // Listens to the changes coming from the client
        };

        // If the Window is ready than send the connection status to it
        if (ready) {
            connectFunc();
        }
        connectedFunc = connectFunc;
    });
    // When the script starts running in the window set the ready variable
    ipc.on('ready', (ev, mesg) => {
        console.log('NetworkTables is ready');
        ready = mainWindow != null;

        // Remove old Listener
        client.removeListener(clientDataListener);

        // Add new listener with immediate callback
        client.addListener(clientDataListener, true);

        // Send connection message to the window if if the message is ready
        if (connectedFunc) connectedFunc();
    });
    // When the user chooses the address of the bot than try to connect
    ipc.on('connect', (ev, address, port) => {
        var d = new Date();
        console.log('time: ' + d.getSeconds() + '.' + d.getMilliseconds());
        console.log(`Trying to connect to ${address}` + (port ? ':' + port : ''));
        console.log();
        let callback = (connected, err) => {
            console.log('Sending status...');
            mainWindow.webContents.send('connected', connected);
        };
        console.log("address: " + address);
        if (port) {
            client.start(callback, address, port);
        } else {
            client.start(callback, address);
        }
    });
    ipc.on('add', (ev, mesg) => {
        console.log("inside ipc.on(add)");
        console.log(client.Assign(mesg.val, mesg.key, true));
        console.log(mesg);
    });
    ipc.on('update', (ev, mesg) => {
        console.log("inside ipc.on(update)");
        console.log(client.isConnected());
        console.log(client.Update(mesg.id, mesg.val));
        console.log(mesg);
    });
    ipc.on('windowError', (ev, error) => {
        console.log(error);
    });
    // Create the browser window.
    mainWindow = new BrowserWindow({
        width: 1500,
        height: 900,
        // 1366x570 is a good standard height, but you may want to change this to fit your DriverStation's screen better.
        // It's best if the dashboard takes up as much space as possible without covering the DriverStation application.
        // The window is closed until the python server is ready
        show: false,
        icon: __dirname + '/../images/icon.png'
    });
    // Move window to top right of screen.
    mainWindow.setPosition(0, 0);
    // mainWindow.setPosition(940, 0);
    // Load window.
    mainWindow.loadURL(`file://${__dirname}/index.html`);
    // Once the python server is ready, load window contents.
    mainWindow.once('ready-to-show', () => {
        console.log('main window is ready to be shown');
        window = mainWindow;
        mainWindow.webContents.reload();
        mainWindow.show();
    });

    // Remove menu
    //mainWindow.setMenu(null);
    // Emitted when the window is closed.
    mainWindow.on('closed', () => {
        console.log('main window closed');
        // Dereference the window object, usually you would store windows
        // in an array if your app supports multi windows, this is the time
        // when you should delete the corresponding element.
        mainWindow = null;
        ready = false;
        connectedFunc = null;
        client.removeListener(clientDataListener);
    });
    mainWindow.on('unresponsive', () => {
        console.log('Main Window is unresponsive');
    });
    mainWindow.webContents.on('did-fail-load', () => {
        console.log('window failed load');
    });
}
// This method will be called when Electron has finished
// initialization and is ready to create browser windows.
app.on('ready', () => {
    console.log('app is ready');
    createWindow();
});

// Quit when all windows are closed.
app.on('window-all-closed', function () {
    // On OS X it is common for applications and their menu bar
    // to stay active until the user quits explicitly with Cmd + Q.
    // Not like we're creating a consumer application though.
    // Let's just kill it anyway.
    // If you want to restore the standard behavior, uncomment the next line.

    // if (process.platform !== 'darwin')
    app.quit();
});

app.on('quit', function () {
    console.log('Application quit.');
});

app.on('activate', function () {
    // On OS X it's common to re-create a window in the app when the
    // dock icon is clicked and there are no other windows open.
    if (mainWindow == null) createWindow();
});
