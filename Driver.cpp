#include <ntddk.h>
#include <wdf.h>
#include <usb.h>
#include <wdfusb.h>
#include <hidclass.h>
#include <usbioctl.h>

//#pragma comment(lib, "setupapi.lib")
//#pragma comment(lib, "hid.lib")

#pragma warning(disable:4100)
#pragma warning(disable:4101)
#pragma warning(disable:4189)
#pragma warning(disable:4366)

// DualShock 4 HID Input report
typedef struct _DS4_REPORT
{
    BYTE bThumbLX;
    BYTE bThumbLY;
    BYTE bThumbRX;
    BYTE bThumbRY;
    USHORT wButtons;
    BYTE bSpecial;
    BYTE bTriggerL;
    BYTE bTriggerR;

} DS4_REPORT, * PDS4_REPORT;

// DualShock 4 request data
typedef struct _DS4_SUBMIT_REPORT
{
    //
    // sizeof(struct _DS4_SUBMIT_REPORT)
    // 
    ULONG Size;

    //
    // Serial number of target device.
    // 
    ULONG SerialNo;

    //
    // HID Input report
    // 
    DS4_REPORT Report;

} DS4_SUBMIT_REPORT, * PDS4_SUBMIT_REPORT;

#include <pshpack1.h>
// DualShock 4 HID Touchpad structure
typedef struct _DS4_TOUCH
{
    BYTE bPacketCounter;    // timestamp / packet counter associated with touch event
    BYTE bIsUpTrackingNum1; // 0 means down; active low
    // unique to each finger down, so for a lift and repress the value is incremented
    BYTE bTouchData1[3];    // Two 12 bits values (for X and Y) 
    // middle byte holds last 4 bits of X and the starting 4 bits of Y
    BYTE bIsUpTrackingNum2; // second touch data immediately follows data of first touch 
    BYTE bTouchData2[3];    // resolution is 1920x943
} DS4_TOUCH, * PDS4_TOUCH;

// DualShock 4 v1 complete HID Input report
typedef struct _DS4_REPORT_EX
{
    union
    {
        struct
        {
            BYTE bThumbLX;
            BYTE bThumbLY;
            BYTE bThumbRX;
            BYTE bThumbRY;
            USHORT wButtons;
            BYTE bSpecial;
            BYTE bTriggerL;
            BYTE bTriggerR;
            USHORT wTimestamp;
            BYTE bBatteryLvl;
            SHORT wGyroX;
            SHORT wGyroY;
            SHORT wGyroZ;
            SHORT wAccelX;
            SHORT wAccelY;
            SHORT wAccelZ;
            BYTE _bUnknown1[5];
            BYTE bBatteryLvlSpecial;
            // really should have a enum to show everything that this can represent (USB charging, battery level; EXT, headset, microphone connected)
            BYTE _bUnknown2[2];
            BYTE bTouchPacketsN; // 0x00 to 0x03 (USB max)
            DS4_TOUCH sCurrentTouch;
            DS4_TOUCH sPreviousTouch[2];
        } Report;

        UCHAR ReportBuffer[63];
    };
} DS4_REPORT_EX, * PDS4_REPORT_EX;

// DualShock 4 extended report request
typedef struct _DS4_SUBMIT_REPORT_EX
{
    //
     // sizeof(struct _DS4_SUBMIT_REPORT_EX)
     // 
    _In_ ULONG Size;

    //
    // Serial number of target device.
    // 
    _In_ ULONG SerialNo;

    //
    // Full size HID report excluding fixed Report ID.
    // 
    _In_ DS4_REPORT_EX Report;

} DS4_SUBMIT_REPORT_EX, * PDS4_SUBMIT_REPORT_EX;

typedef struct _NEW_DS4_REPORT_INCOME {
    BYTE bytes[64];
} NEW_DS4_REPORT_INCOME, * PNEW_DS4_REPORT_INCOME;
#include <poppack.h>



static const int DS4_REPORT_SIZE = 64;
static const int DS4_QUEUE_FLUSH_PERIOD = 5;

typedef struct _DEVICE_CONTEXT {
    WDFDEVICE WdfDevice;
    WDFQUEUE DefaultQueue;
    WDFQUEUE PendingInputQueue;
    WDFTIMER PendingInputTimer;
    UCHAR _Report[DS4_REPORT_SIZE];
} DEVICE_CONTEXT, * PDEVICE_CONTEXT;

WDF_DECLARE_CONTEXT_TYPE_WITH_NAME(DEVICE_CONTEXT, DeviceGetContext)

extern "C"
DRIVER_INITIALIZE DriverEntry;
EVT_WDF_DRIVER_DEVICE_ADD EvtDeviceAdd;
EVT_WDF_DRIVER_UNLOAD EvtUnload;
EVT_WDF_DEVICE_PREPARE_HARDWARE EvtPrepareHardware;
EVT_WDF_IO_QUEUE_IO_INTERNAL_DEVICE_CONTROL EvtIoInternalDeviceControl;
EVT_WDF_OBJECT_CONTEXT_CLEANUP EvtDeviceContextCleanup;
EVT_WDF_TIMER PendingUsbRequestsTimerFunc;
NTSTATUS BulkOrInterrupt(_URB_BULK_OR_INTERRUPT_TRANSFER* pTransfer, WDFREQUEST Request, WDFDEVICE Device);



extern "C"
NTSTATUS DriverEntry(PDRIVER_OBJECT DriverObject, PUNICODE_STRING RegistryPath) {
    KdPrint(("DriverEntry\n"));
    WDF_DRIVER_CONFIG config;
    WDF_DRIVER_CONFIG_INIT(&config, EvtDeviceAdd);
    config.EvtDriverUnload = EvtUnload;
    ExInitializeDriverRuntime(DrvRtPoolNxOptIn);
    return WdfDriverCreate(DriverObject, RegistryPath, WDF_NO_OBJECT_ATTRIBUTES, &config, WDF_NO_HANDLE);
}

NTSTATUS EvtDeviceAdd(_In_ WDFDRIVER Driver, _Inout_ PWDFDEVICE_INIT DeviceInit) {

    KdPrint(("DeviceAdd\n"));

    NTSTATUS status = STATUS_UNSUCCESSFUL;
    WDF_PNPPOWER_EVENT_CALLBACKS pnpPowerCallbacks;
    WDF_OBJECT_ATTRIBUTES deviceAttributes;
    WDFDEVICE device;
    WDF_OBJECT_ATTRIBUTES attributes;
    PDEVICE_CONTEXT deviceContext;
    WDF_TIMER_CONFIG timerConfig;
    WDF_OBJECT_ATTRIBUTES timerAttribs;
    WDF_IO_QUEUE_CONFIG PendingInputQueueConfig;
    WDF_IO_QUEUE_CONFIG DefaultQueueConfig;
    
    do
    {
        // Set As Filter =======================================
        WdfFdoInitSetFilter(DeviceInit);



        // Create Power Callbacks ==============================
        WDF_PNPPOWER_EVENT_CALLBACKS_INIT(&pnpPowerCallbacks);
        pnpPowerCallbacks.EvtDevicePrepareHardware = EvtPrepareHardware;
        WdfDeviceInitSetPnpPowerEventCallbacks(DeviceInit, &pnpPowerCallbacks);



        // Create Context ======================================
        WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE(&deviceAttributes, DEVICE_CONTEXT);
        deviceAttributes.EvtCleanupCallback = EvtDeviceContextCleanup;
        status = WdfDeviceCreate(&DeviceInit, &deviceAttributes, &device);
        if (!NT_SUCCESS(status)) break;
        deviceContext = DeviceGetContext(device);
        deviceContext->WdfDevice = device;



        // Create Timer ========================================
        WDF_TIMER_CONFIG_INIT_PERIODIC(&timerConfig, PendingUsbRequestsTimerFunc, DS4_QUEUE_FLUSH_PERIOD);
        WDF_OBJECT_ATTRIBUTES_INIT(&timerAttribs);
        timerAttribs.ParentObject = deviceContext->WdfDevice;
        if (!NT_SUCCESS(status = WdfTimerCreate(&timerConfig, &timerAttribs, &deviceContext->PendingInputTimer))) break;



        // Create Pending Input Queue ==========================
        WDF_OBJECT_ATTRIBUTES_INIT(&attributes);
        attributes.ParentObject = device;
        WDF_IO_QUEUE_CONFIG_INIT(&PendingInputQueueConfig, WdfIoQueueDispatchManual);
        status = WdfIoQueueCreate(device, &PendingInputQueueConfig, WDF_NO_OBJECT_ATTRIBUTES, &deviceContext->PendingInputQueue);
        if (!NT_SUCCESS(status)) { KdPrint(("IoQueueCreate - Default Queue failed\n")); break; }



        // Create Default IO Queue =============================
        WDF_OBJECT_ATTRIBUTES_INIT(&attributes);
        attributes.ParentObject = device;
        WDF_IO_QUEUE_CONFIG_INIT_DEFAULT_QUEUE(&DefaultQueueConfig, WdfIoQueueDispatchParallel);
        DefaultQueueConfig.EvtIoInternalDeviceControl = EvtIoInternalDeviceControl;
        status = WdfIoQueueCreate(device, &DefaultQueueConfig, WDF_NO_OBJECT_ATTRIBUTES, &deviceContext->DefaultQueue);
        if (!NT_SUCCESS(status)) { KdPrint(("IoQueueCreate - Default Queue failed\n")); break; }



    } while (FALSE);

    return STATUS_SUCCESS;
}

NTSTATUS EvtPrepareHardware(WDFDEVICE Device, WDFCMRESLIST ResourcesRaw, WDFCMRESLIST ResourcesTranslated) {

    KdPrint(("PrepareHardware\n"));

    NTSTATUS status;
    PDEVICE_CONTEXT deviceContext = DeviceGetContext(Device);

    // Set default HID input report (everything zero`d)
    UCHAR DefaultHidReport[DS4_REPORT_SIZE] =
    {
        0x01, 0x82, 0x7F, 0x7E, 0x80, 0x08, 0x00, 0x58,
        0x00, 0x00, 0xFD, 0x63, 0x06, 0x03, 0x00, 0xFE,
        0xFF, 0xFC, 0xFF, 0x79, 0xFD, 0x1B, 0x14, 0xD1,
        0xE9, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1B, 0x00,
        0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x80,
        0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00,
        0x80, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00,
        0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00
    };

    // Initialize HID reports to defaults
    RtlCopyMemory(&deviceContext->_Report, DefaultHidReport, DS4_REPORT_SIZE);
    WdfTimerStart(deviceContext->PendingInputTimer, DS4_QUEUE_FLUSH_PERIOD);

    // \GLOBAL??\HID#VID_054C & PID_09CC & MI_03#7 & 17773ab2 & 0 & 0000#{4d1e55b2 - f16f - 11cf - 88cb - 001111000030}
    // find this path automatically ^ (maybe send it requests n shit)

    return STATUS_SUCCESS;
}

VOID EvtDeviceContextCleanup(WDFOBJECT ObjectContext) {
    KdPrint(("ContextCleanup\n"));
}

VOID EvtUnload(_In_ WDFDRIVER DriverObject) {
    KdPrint(("Unload\n"));
}

VOID EvtIoInternalDeviceControl(WDFQUEUE Queue, WDFREQUEST Request, size_t OutputSize, size_t InputSize, ULONG ControlCode) {

    NTSTATUS status = STATUS_INVALID_PARAMETER;
    PIRP irp;
    PURB urb;
    PIO_STACK_LOCATION irpStack;
    PDEVICE_CONTEXT deviceContext = DeviceGetContext(WdfIoQueueGetDevice(Queue));
    irp = WdfRequestWdmGetIrp(Request);
    irpStack = IoGetCurrentIrpStackLocation(irp);
    urb = static_cast<PURB>(URB_FROM_IRP(irp));

    switch (ControlCode)
    {

    case IOCTL_HID_GET_INPUT_REPORT:
        KdPrint(("IOCTL_HID_GET_INPUT_REPORT\n"));
        break;

    case IOCTL_HID_GET_OUTPUT_REPORT:
        KdPrint(("IOCTL_HID_GET_OUTPUT_REPORT\n"));
        break;

    case IOCTL_INTERNAL_USB_SUBMIT_URB:
        KdPrint(("IOCTL_INTERNAL_USB_SUBMIT_URB -----\t"));

        switch (urb->UrbHeader.Function)
        {
        case URB_FUNCTION_CONTROL_TRANSFER:
            KdPrint(("URB_FUNCTION_CONTROL_TRANSFER\n"));
            break;

        case URB_FUNCTION_CONTROL_TRANSFER_EX:
            KdPrint(("URB_FUNCTION_CONTROL_TRANSFER_EX\n"));
            break;

        case URB_FUNCTION_BULK_OR_INTERRUPT_TRANSFER:
            //KdPrint(("URB_FUNCTION_BULK_OR_INTERRUPT_TRANSFER\t"));
            status = BulkOrInterrupt(&urb->UrbBulkOrInterruptTransfer, Request, deviceContext->WdfDevice);
            break;

        case URB_FUNCTION_SELECT_CONFIGURATION:
            KdPrint(("URB_FUNCTION_SELECT_CONFIGURATION\n"));
            break;

        case URB_FUNCTION_SELECT_INTERFACE:
            KdPrint(("URB_FUNCTION_SELECT_INTERFACE\n"));
            break;

        case URB_FUNCTION_GET_DESCRIPTOR_FROM_DEVICE:
            KdPrint(("URB_FUNCTION_GET_DESCRIPTOR_FROM_DEVICE\n"));
            break;

        case URB_FUNCTION_GET_STATUS_FROM_DEVICE:
            KdPrint(("URB_FUNCTION_GET_STATUS_FROM_DEVICE\n"));
            break;

        case URB_FUNCTION_ABORT_PIPE:
            KdPrint(("URB_FUNCTION_ABORT_PIPE\n"));
            break;

        case URB_FUNCTION_CLASS_INTERFACE:
            KdPrint(("URB_FUNCTION_CLASS_INTERFACE\n"));
            break;

        case URB_FUNCTION_GET_DESCRIPTOR_FROM_INTERFACE:
            KdPrint(("URB_FUNCTION_GET_DESCRIPTOR_FROM_INTERFACE\n"));
            break;

        default:
            break;
        }
        break;

    case IOCTL_INTERNAL_USB_GET_PORT_STATUS:
        KdPrint(("IOCTL_INTERNAL_USB_GET_PORT_STATUS\n"));
        break;

    case IOCTL_INTERNAL_USB_RESET_PORT:
        KdPrint(("IOCTL_INTERNAL_USB_RESET_PORT\n"));
        break;

    case IOCTL_INTERNAL_USB_SUBMIT_IDLE_NOTIFICATION:
        KdPrint(("IOCTL_INTERNAL_USB_SUBMIT_IDLE_NOTIFICATION\n"));
        break;
    }

    if (status != STATUS_PENDING) {
        WDF_REQUEST_SEND_OPTIONS sendOptions;
        WDF_REQUEST_SEND_OPTIONS_INIT(&sendOptions, WDF_REQUEST_SEND_OPTION_SEND_AND_FORGET);
        if (WdfRequestSend(Request, WdfDeviceGetIoTarget(WdfIoQueueGetDevice(Queue)), &sendOptions) == FALSE) {
            KdPrint(("Failed Send\t"));
            NTSTATUS status2 = WdfRequestGetStatus(Request);
            WdfRequestComplete(Request, status2);
        }
    }
}

NTSTATUS BulkOrInterrupt(_URB_BULK_OR_INTERRUPT_TRANSFER* pTransfer, WDFREQUEST Request, WDFDEVICE Device) {

    NTSTATUS status = STATUS_SUCCESS;

    if (pTransfer->TransferBufferMDL == NULL) {


        PDEVICE_CONTEXT deviceContext = DeviceGetContext(Device);
        PVOID NewReport = pTransfer->TransferBuffer;

        // Get pending IRP
        PIRP pendingIrp = WdfRequestWdmGetIrp(Request);

        // Get USB request block
        const auto urb = static_cast<PURB>(URB_FROM_IRP(pendingIrp));

        // Get transfer buffer
        const auto buffer = static_cast<PUCHAR>(urb->UrbBulkOrInterruptTransfer.TransferBuffer);

        // Set correct buffer size
        urb->UrbBulkOrInterruptTransfer.TransferBufferLength = DS4_REPORT_SIZE;

        // Cast to expected struct
        const auto pSubmit = static_cast<PDS4_SUBMIT_REPORT>(NewReport);

        KdPrint(("Report Size: %lu\t", pSubmit->Size));

        // "Old" API which only allows to update partial report
        //if (pSubmit->Size == sizeof(DS4_SUBMIT_REPORT)) {
        //    KdPrint(("Partial Report\n"));
        //    PDS4_SUBMIT_REPORT nRep = static_cast<PDS4_SUBMIT_REPORT>(NewReport);
        //
        //    nRep->Report.bThumbLX = 0xFF;
        //    nRep->Report.bThumbLY = 0xFF;
        //
        //    RtlCopyMemory(&deviceContext->_Report[1], &nRep->Report, sizeof((static_cast<PDS4_SUBMIT_REPORT>(NewReport))->Report));
        //}

        // "Extended" API allowing complete report update
        //if (pSubmit->Size == sizeof(DS4_SUBMIT_REPORT_EX)) {
        //    KdPrint(("Extended Report\n"));
        //    PDS4_SUBMIT_REPORT_EX nRep = static_cast<PDS4_SUBMIT_REPORT_EX>(NewReport);
        //
        //    nRep->Report.ReportBuffer[0] = 0xFF;
        //    nRep->Report.ReportBuffer[1] = 0xFF;
        //
        //    RtlCopyMemory(&deviceContext->_Report[1], &(static_cast<PDS4_SUBMIT_REPORT_EX>(NewReport))->Report, sizeof((static_cast<PDS4_SUBMIT_REPORT_EX>(NewReport))->Report));
        //}

        // If this is INPUT
        if (pTransfer->TransferFlags & USBD_TRANSFER_DIRECTION_IN) {
            status = WdfRequestForwardToIoQueue(Request, deviceContext->PendingInputQueue);
            return (NT_SUCCESS(status)) ? STATUS_PENDING : status;
        }
    }

    return status;
}

VOID PendingUsbRequestsTimerFunc(_In_ WDFTIMER Timer) {

    PDEVICE_CONTEXT deviceContext = DeviceGetContext(WdfTimerGetParentObject(Timer));
    WDFREQUEST usbRequest;
    NTSTATUS status = WdfIoQueueRetrieveNextRequest(deviceContext->PendingInputQueue, &usbRequest);

    if (!NT_SUCCESS(status))
        return;

    if (NT_SUCCESS(status)) {

        KdPrint(("Completing report\n"));

        // Get pending IRP
        PIRP pendingIrp = WdfRequestWdmGetIrp(usbRequest);
        PIO_STACK_LOCATION irpStack = IoGetCurrentIrpStackLocation(pendingIrp);

        // Get USB request block
        PURB urb = static_cast<PURB>(irpStack->Parameters.Others.Argument1);

        // Get transfer buffer
        PUCHAR buffer = static_cast<PUCHAR>(urb->UrbBulkOrInterruptTransfer.TransferBuffer);

        // Set buffer length to report size
        urb->UrbBulkOrInterruptTransfer.TransferBufferLength = DS4_REPORT_SIZE;

        // Copy cached report to transfer buffer
        if (buffer)
            RtlCopyMemory(buffer, deviceContext->_Report, DS4_REPORT_SIZE);

        // Complete pending request
        WdfRequestComplete(usbRequest, status);
    }

    // old one
    //// Get USB request block
    //const auto urb = static_cast<PURB>(URB_FROM_IRP(pendingIrp));
    //
    //PVOID voidBuffer = urb->UrbBulkOrInterruptTransfer.TransferBuffer;
    //
    //// Get transfer buffer
    //const auto buffer = static_cast<PUCHAR>(voidBuffer);
    //
    //
    //// 
    //PDS4_SUBMIT_REPORT ds4Submit = (PDS4_SUBMIT_REPORT)buffer;
    //
    //
    //// Set correct buffer size
    //urb->UrbBulkOrInterruptTransfer.TransferBufferLength = DS4_REPORT_SIZE;
    //
    //
    //// Cast to expected struct
    //const auto pSubmit = static_cast<PDS4_SUBMIT_REPORT>(voidBuffer);
    //
    ////
    //// "Old" API which only allows to update partial report
    //// 
    //if (pSubmit->Size == sizeof(DS4_SUBMIT_REPORT)) {
    //    RtlCopyMemory(&deviceContext->_Report[1], &(static_cast<PDS4_SUBMIT_REPORT>(voidBuffer))->Report, sizeof((static_cast<PDS4_SUBMIT_REPORT>(voidBuffer))->Report));
    //}
    //
    ////
    //// "Extended" API allowing complete report update
    //// 
    //if (pSubmit->Size == sizeof(DS4_SUBMIT_REPORT_EX)) {
    //    RtlCopyMemory(&deviceContext->_Report[1], &(static_cast<PDS4_SUBMIT_REPORT_EX>(voidBuffer))->Report, sizeof((static_cast<PDS4_SUBMIT_REPORT_EX>(voidBuffer))->Report));
    //}
    //
    ////deviceContext->_Report[1] = 0xFF;
    //
    //if (buffer)
    //    RtlCopyMemory(buffer, deviceContext->_Report, DS4_REPORT_SIZE);
    //
    //// Complete pending request
    //WdfRequestComplete(usbRequest, status);
}