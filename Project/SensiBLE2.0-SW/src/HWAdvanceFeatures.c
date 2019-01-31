#include "HWAdvanceFeatures.h"
#include "sensible20_accelero.h"
#include "sensible20_port_exp.h"
#include "inertial_app.h"
#include "sensible_services.h"
#include "LIS2DW12_ACC_driver.h"
#include "acceleroSamplesProcessing.h"

/* Exported variables ---------------------------------------------------------*/
uint32_t HWAdvanceFeaturesStatus = 0;

/* Private Variables -------------------------------------------------------------*/
static float DefaultAccODR;
static BOOL MultipleAccEventEnabled = 0;

static void processAccFifo(void);
static void processAccEvents(void);

void HWFeaturesCallback(uint8_t irqPin)
{
    switch(irqPin){
    case LIS2DW12_INT1_PIN:
        processAccEvents();
        break;
    case LIS2DW12_INT2_PIN:
        processAccFifo();
        break;
    default:
        break;
    }
}

/**
  * @brief  This function Reads the default Acceleration Output Data Rate
  * @param  None
  * @retval None
  */
void InitHWFeatures(void)
{
    /* Read the Default Output Data Rate for Accelerometer */
    BSP_ACCELERO_Get_ODR(ACCELERO_handle, &DefaultAccODR);
}

/**
  * @brief  This function disables all the HW's Features
  * @param  None
  * @retval None
  */
void DisableHWFeatures(void)
{
    if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_PEDOMETER)) {
        DisableSWPedometer();
    }
    
    if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_TILT)){
        DisableSWTilt();
    }
    
    if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_FREE_FALL)) {
        DisableHWFreeFall();
    }

    if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_DOUBLE_TAP)) {
        DisableHWDoubleTap();
    }

    if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_SINGLE_TAP)) {
        DisableHWSingleTap();
    }

    if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_WAKE_UP)) {
        DisableHWWakeUp();
    }
    
    if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_6DORIENTATION)) {
        DisableHWOrientation6D();
    }
}

/**
  * @brief  This function enables the SW's pedometer
  * @param  None
  * @retval None
  */
void EnableSWPedometer(void)
{
    if(BSP_ACCELERO_FIFO_Set_ODR_Value_Ext(ACCELERO_handle, 50.0f) != COMPONENT_OK){
        goto fail;
    }
    
    if(BSP_ACCELERO_FIFO_Set_Watermark_Level_Ext(ACCELERO_handle, LIS2DW12_FIFO_SIZE) != COMPONENT_OK){
        goto fail;
    }
    
    if(BSP_ACCELERO_FIFO_Set_INT2_FIFO_Treshold_Ext(ACCELERO_handle, ENABLE) != COMPONENT_OK){
        goto fail;
    }
    
    if(BSP_ACCELERO_FIFO_Set_Mode_Ext(ACCELERO_handle, LIS2DW12_ACC_FMODE_STREAM) != COMPONENT_OK){
        goto fail;
    }
    
    W2ST_ON_HW_FEATURE(W2ST_HWF_PEDOMETER);
    
fail: return;
}

/**
  * @brief  This function disables the SW's pedometer
  * @param  None
  * @retval None
  */
void DisableSWPedometer(void)
{
    W2ST_OFF_HW_FEATURE(W2ST_HWF_PEDOMETER);
    
    BSP_ACCELERO_FIFO_Set_Mode_Ext(ACCELERO_handle, LIS2DW12_ACC_FMODE_BYPASS);
    BSP_ACCELERO_FIFO_Set_INT2_FIFO_Treshold_Ext(ACCELERO_handle, DISABLE);
    /* Set the Output Data Rate to Default value */
    BSP_ACCELERO_Set_ODR_Value(ACCELERO_handle, DefaultAccODR);
}

/**
  * @brief  This function resets the SW's pedometer steps counter
  * @param  None
  * @retval None
  */
void ResetSWPedometer(void)
{
    resetSteps();
}

/**
  * @brief  This function retunrs the SW's pedometer steps counter value
  * @param  None
  * @retval uint16_t Steps Counter
  */
uint16_t GetStepSWPedometer(void)
{
    return getSteps();
}

/**
  * @brief  This function enables the SW's tilt detection
  * @param  None
  * @retval None
  */
void EnableSWTilt(void)
{
    if(BSP_ACCELERO_FIFO_Set_ODR_Value_Ext(ACCELERO_handle, 50.0f) != COMPONENT_OK){
        goto fail;
    }
    
    if(BSP_ACCELERO_FIFO_Set_Watermark_Level_Ext(ACCELERO_handle, LIS2DW12_FIFO_SIZE) != COMPONENT_OK){
        goto fail;
    }
    
    if(BSP_ACCELERO_FIFO_Set_INT2_FIFO_Treshold_Ext(ACCELERO_handle, ENABLE) != COMPONENT_OK){
        goto fail;
    }
    
    if(BSP_ACCELERO_FIFO_Set_Mode_Ext(ACCELERO_handle, LIS2DW12_ACC_FMODE_STREAM) != COMPONENT_OK){
        goto fail;
    }
    
    W2ST_ON_HW_FEATURE(W2ST_HWF_TILT);
    
fail: return;
}

/**
  * @brief  This function disables the SW's tilt detection
  * @param  None
  * @retval None
  */
void DisableSWTilt(void)
{
    W2ST_OFF_HW_FEATURE(W2ST_HWF_TILT);
    
    BSP_ACCELERO_FIFO_Set_Mode_Ext(ACCELERO_handle, LIS2DW12_ACC_FMODE_BYPASS);
    BSP_ACCELERO_FIFO_Set_INT2_FIFO_Treshold_Ext(ACCELERO_handle, DISABLE);
    /* Set the Output Data Rate to Default value */
    BSP_ACCELERO_Set_ODR_Value(ACCELERO_handle, DefaultAccODR);
}

/**
  * @brief  This function enables the HW's 6D Orientation
  * @param  None
  * @retval None
  */
void EnableHWOrientation6D(void)
{
    /* Disable all the HW features before */
    if(!MultipleAccEventEnabled){
        DisableHWFeatures();
    }
    
    /* Enable 6D orientation detection */
    if(BSP_ACCELERO_Enable_6D_Orientation_Ext(ACCELERO_handle) == COMPONENT_OK){
        W2ST_ON_HW_FEATURE(W2ST_HWF_6DORIENTATION);
    }
}

/**
  * @brief  This function disables the HW's 6D Orientation
  * @param  None
  * @retval None
  */
void DisableHWOrientation6D(void)
{
    /* Disable 6D orientation detection */
    if(BSP_ACCELERO_Disable_6D_Orientation_Ext(ACCELERO_handle) == COMPONENT_OK){
        W2ST_OFF_HW_FEATURE(W2ST_HWF_6DORIENTATION);
    }
    
    /* Set the Output Data Rate to Default value */
    BSP_ACCELERO_Set_ODR_Value(ACCELERO_handle, DefaultAccODR);
}

/**
  * @brief  This function eturns the HW's 6D Orientation result
  * @param  None
  * @retval AccEventType 6D Orientation Found
  */
AccEventType GetHWOrientation6D(void)
{  
    AccEventType OrientationResult = ACC_NOT_USED;
    uint8_t xl = 0;
    uint8_t xh = 0;
    uint8_t yl = 0;
    uint8_t yh = 0;
    uint8_t zl = 0;
    uint8_t zh = 0;

    if(BSP_ACCELERO_Get_6D_Orientation_XL_Ext(ACCELERO_handle, &xl) == COMPONENT_ERROR){
        goto fail;
    }

    if(BSP_ACCELERO_Get_6D_Orientation_XH_Ext(ACCELERO_handle, &xh) == COMPONENT_ERROR){
        goto fail;
    }

    if(BSP_ACCELERO_Get_6D_Orientation_YL_Ext(ACCELERO_handle, &yl) == COMPONENT_ERROR){
        goto fail;
    }

    if(BSP_ACCELERO_Get_6D_Orientation_YH_Ext(ACCELERO_handle, &yh) == COMPONENT_ERROR){
        goto fail;
    }

    if(BSP_ACCELERO_Get_6D_Orientation_ZL_Ext(ACCELERO_handle, &zl) == COMPONENT_ERROR){
        goto fail;
    }

    if(BSP_ACCELERO_Get_6D_Orientation_ZH_Ext(ACCELERO_handle, &zh) == COMPONENT_ERROR){
        goto fail;
    }

    if(xl == 0 && yl == 0 && zl == 0 && xh == 0 && yh == 1 && zh == 0){
        OrientationResult = ACC_6D_OR_RIGTH;
    } else if (xl == 1 && yl == 0 && zl == 0 && xh == 0 && yh == 0 && zh == 0){
        OrientationResult = ACC_6D_OR_TOP;
    } else if (xl == 0 && yl == 0 && zl == 0 && xh == 1 && yh == 0 && zh == 0){
        OrientationResult = ACC_6D_OR_BOTTOM;
    } else if (xl == 0 && yl == 1 && zl == 0 && xh == 0 && yh == 0 && zh == 0){
        OrientationResult = ACC_6D_OR_LEFT;
    } else if (xl == 0 && yl == 0 && zl == 0 && xh == 0 && yh == 0 && zh == 1){
        OrientationResult = ACC_6D_OR_UP;
    } else if (xl == 0 && yl == 0 && zl == 1 && xh == 0 && yh == 0 && zh == 0){
        OrientationResult = ACC_6D_OR_DOWN;
    } else {
        goto fail;
    }
    
    return OrientationResult;
    
fail: return ACC_NOT_USED;
}

/**
  * @brief  This function enables the HW's Wake Up Detection
  * @param  None
  * @retval None
  */
void EnableHWWakeUp(void)
{
    /* Disable all the HW features before */
    if(!MultipleAccEventEnabled){
        DisableHWFeatures();
    }
    /* Enable Wake up detection */
    if(BSP_ACCELERO_Enable_Wake_Up_Detection_Ext(ACCELERO_handle) == COMPONENT_OK){
        W2ST_ON_HW_FEATURE(W2ST_HWF_WAKE_UP);
    }
}

/**
  * @brief  This function disables the HW's Wake Up Detection
  * @param  None
  * @retval None
  */
void DisableHWWakeUp(void)
{
    /* Disable Wake up detection */
    if(BSP_ACCELERO_Disable_Wake_Up_Detection_Ext(ACCELERO_handle) == COMPONENT_OK){
        W2ST_OFF_HW_FEATURE(W2ST_HWF_WAKE_UP);
    }

    /* Set the Output Data Rate to Default value */
    BSP_ACCELERO_Set_ODR_Value(ACCELERO_handle, DefaultAccODR);
}

/**
  * @brief  This function enables the HW's Free Fall Detection
  * @param  None
  * @retval None
  */
void EnableHWFreeFall(void)
{
    /* Disable all the HW features before */
    if(!MultipleAccEventEnabled){
        DisableHWFeatures();
    }
    
    /* Enable Free Fall detection */
    if(BSP_ACCELERO_Enable_Free_Fall_Detection_Ext(ACCELERO_handle) == COMPONENT_OK){
        W2ST_ON_HW_FEATURE(W2ST_HWF_FREE_FALL);
        BSP_ACCELERO_Set_Free_Fall_Threshold_Ext(ACCELERO_handle, LIS2DW12_ACC_FF_THS_7);
    } 
}

/**
  * @brief  This function disables the HW's Free Fall Detection
  * @param  None
  * @retval None
  */
void DisableHWFreeFall(void)
{
    /* Disable Free Fall detection */
    if(BSP_ACCELERO_Disable_Free_Fall_Detection_Ext(ACCELERO_handle) == COMPONENT_OK){
        W2ST_OFF_HW_FEATURE(W2ST_HWF_FREE_FALL);
    }

    /* Set the Output Data Rate to Default value */
    BSP_ACCELERO_Set_ODR_Value(ACCELERO_handle, DefaultAccODR);
}

/**
  * @brief  This function enables the HW's Double Tap Detection
  * @param  None
  * @retval None
  */
void EnableHWDoubleTap(void)
{
    /* Disable all the HW features before */
    if(!MultipleAccEventEnabled){
        DisableHWFeatures();
    }

    /* Enable Double Tap detection */
    if(BSP_ACCELERO_Enable_Double_Tap_Detection_Ext(ACCELERO_handle) == COMPONENT_OK){
        W2ST_ON_HW_FEATURE(W2ST_HWF_DOUBLE_TAP);
    }
}

/**
  * @brief  This function disables the HW's Double Tap Detection
  * @param  None
  * @retval None
  */
void DisableHWDoubleTap(void)
{
    /* Disable Double Tap detection */
    if(BSP_ACCELERO_Disable_Double_Tap_Detection_Ext(ACCELERO_handle) == COMPONENT_OK){
        W2ST_OFF_HW_FEATURE(W2ST_HWF_DOUBLE_TAP);
    }

    /* Set the Output Data Rate to Default value */
    BSP_ACCELERO_Set_ODR_Value(ACCELERO_handle, DefaultAccODR);
}

/**
  * @brief  This function enables the HW's Single Tap Detection
  * @param  None
  * @retval None
  */
void EnableHWSingleTap(void)
{
    /* Disable all the HW features before */
    if(!MultipleAccEventEnabled){
        DisableHWFeatures();
    }
    
    /* Enable Single Tap detection */
    if(BSP_ACCELERO_Enable_Single_Tap_Detection_Ext(ACCELERO_handle) == COMPONENT_OK){
        W2ST_ON_HW_FEATURE(W2ST_HWF_SINGLE_TAP);
    }  
}

/**
  * @brief  This function disables the HW's Single Tap Detection
  * @param  None
  * @retval None
  */
void DisableHWSingleTap(void)
{
    /* Disable Single Tap detection */
    if(BSP_ACCELERO_Disable_Single_Tap_Detection_Ext(ACCELERO_handle) == COMPONENT_OK){
        W2ST_OFF_HW_FEATURE(W2ST_HWF_SINGLE_TAP);
    }
    
    /* Set the Output Data Rate to Default value */
    BSP_ACCELERO_Set_ODR_Value(ACCELERO_handle, DefaultAccODR);
}

/**
  * @brief  This function enables the multiple HW's events
  * @param  None
  * @retval None
  */
void EnableHWMultipleEvents(void)
{
    DisableHWFeatures();

    BSP_ACCELERO_Sensor_Enable(ACCELERO_handle);
    
    MultipleAccEventEnabled = TRUE;

    EnableHWFreeFall();
    EnableHWOrientation6D();
    
    /* Do not change the enable sequenze of the Single and Double-Tap events */
    /* It depends on the Accelero features */
    EnableHWSingleTap();
    EnableHWDoubleTap();

//    EnableHWWakeUp();
    
    EnableSWPedometer();
    EnableSWTilt();
    
    AccEvent_Notify(GetStepSWPedometer(), 3);
    
    W2ST_ON_HW_FEATURE(W2ST_HWF_MULTIPLE_EVENTS);
}

/**
  * @brief  This function disables the multiple HW's events
  * @param  None
  * @retval None
  */
void DisableHWMultipleEvents(void)
{
    DisableHWFeatures();
    MultipleAccEventEnabled = FALSE;
    W2ST_OFF_HW_FEATURE(W2ST_HWF_MULTIPLE_EVENTS);
    
    BSP_ACCELERO_Sensor_Disable(ACCELERO_handle);
}

static void processAccEvents(void)
{
    uint8_t intSrcReg = 0;
    
    BSP_ACCELERO_Read_All_Int_Src_Register_Ext(ACCELERO_handle, &intSrcReg);
    
    if((W2ST_CHECK_HW_FEATURE(W2ST_HWF_FREE_FALL)) ||
       (W2ST_CHECK_HW_FEATURE(W2ST_HWF_MULTIPLE_EVENTS)))
    {
        /* Check if the Free Fall interrupt flag is set */
        if(intSrcReg & ALL_INT_REG_FF_IA){
            AccEvent_Notify(ACC_FREE_FALL, 2);
        }
    }
    
    if((W2ST_CHECK_HW_FEATURE(W2ST_HWF_DOUBLE_TAP)) ||
       (W2ST_CHECK_HW_FEATURE(W2ST_HWF_MULTIPLE_EVENTS)))
    {
        /* Check if the Double Tap interrupt flag is set */
        if(intSrcReg & ALL_INT_REG_DOUBLE_TAP){
            AccEvent_Notify(ACC_DOUBLE_TAP, 2);
        }
    }
    
    if((W2ST_CHECK_HW_FEATURE(W2ST_HWF_SINGLE_TAP)) ||
       (W2ST_CHECK_HW_FEATURE(W2ST_HWF_MULTIPLE_EVENTS)))
    {
        /* Check if the Single Tap interrupt flag is set */
        if(intSrcReg & ALL_INT_REG_SINGLE_TAP){
            AccEvent_Notify(ACC_SINGLE_TAP, 2);
        }
    }
    
    if((W2ST_CHECK_HW_FEATURE(W2ST_HWF_6DORIENTATION)) ||
       (W2ST_CHECK_HW_FEATURE(W2ST_HWF_MULTIPLE_EVENTS)))
    {
        /* Check if the 6D Orientation interrupt flag is set */
        if(intSrcReg & ALL_INT_REG_6D_IA){
            AccEventType orientation = GetHWOrientation6D();
            AccEvent_Notify(orientation, 2);
        }
    }

//    if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_WAKE_UP)) {
//        /* Check if the Wake Up interrupt flag is set */
//        if(intSrcReg & ALL_INT_REG_WU_IA){
//            AccEvent_Notify(ACC_WAKE_UP, 2);
//        }
//    }
}

static void processAccFifo(void)
{
    uint8_t status = 0;
    BSP_ACCELERO_FIFO_Get_Full_Status_Ext(ACCELERO_handle, &status);
    if(status){
        flushAcceleroFifo(ACCELERO_handle);
        
        if((W2ST_CHECK_HW_FEATURE(W2ST_HWF_PEDOMETER)) ||
            (W2ST_CHECK_HW_FEATURE(W2ST_HWF_MULTIPLE_EVENTS)))
        {
            if(lookForSteps()){
                AccEvent_Notify(GetStepSWPedometer(), 3);//2);
            }
        }
        
        if((W2ST_CHECK_HW_FEATURE(W2ST_HWF_TILT)) ||
            (W2ST_CHECK_HW_FEATURE(W2ST_HWF_MULTIPLE_EVENTS)))
        {
            if(lookForTilt()){
                AccEvent_Notify(ACC_TILT, 2);
            }
        }
    }
}
