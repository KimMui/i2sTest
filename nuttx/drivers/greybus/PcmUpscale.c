// PcmUpscale.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "rescale_pcm.h"

#define MAX_SAMPLE_TO_READ 576

//maximum number of channels for upscaling
#define MAX_CHANNELS 6

// Target Frequency we are upscaling to
#define MATCH_FREQ 48000

#define RANDOMIZE_INPUT_SIZE

static Int16 tmpBuf[4096];
UINT32 g_OutFileSize = 0;

Uint32 File_Write(INT16 *pcmData, Uint32 nSamples)
{
    UINT32 SizeOfBuffer = nSamples * sizeof(INT16);

    if (MMSYSERR_NOERROR != WriteWaveFile(SizeOfBuffer, (PBYTE)pcmData))
    {
        _tprintf(TEXT("WriteWaveFile Error\r\n"));
    }

    g_OutFileSize += SizeOfBuffer;

    return nSamples;
}

int _tmain(int argc, _TCHAR* argv[])
{
    HANDLE fh = NULL;
    INT16 *pInBuff;
    INT16 *pOutBuff;
    //TCHAR InFileName[MAX_PATH] = TEXT("C:\\11025.wav");
    
    TCHAR InFileName[MAX_PATH] = TEXT("C:\\Work\\PcmUpscale\\SampleFiles\\3ChanSTest8K.wav");
    //TCHAR InFileName[MAX_PATH] = TEXT("C:\\Work\\PcmUpscale\\SampleFiles\\8K.wav");
    

    TCHAR OutFileName[MAX_PATH];
    DWORD inBufferSize;
    BYTE *pinBuffer;
    PWAVEFORMATEX pWFX;
    uint32_t InSamplesRead = 0;
    int InSamples = 0;
    uint32_t nDst;

    uint32_t SkipCount = 0;

    if(MMSYSERR_NOERROR != ReadWaveFile(InFileName, &pWFX, &inBufferSize, &pinBuffer))
    {
        return 0;
    }

    if(16 != pWFX->wBitsPerSample)
    {
        _tprintf(TEXT("Only 16BPP Wave files suppoted \r\n"));
        return 0;
    }

    if(MAX_CHANNELS < pWFX->nChannels)
    {
        _tprintf(TEXT("No more than 6 channels supported \r\n"));
        return 0;
    }

    _tcsncpy_s (OutFileName, (sizeof(OutFileName)/sizeof(OutFileName[0])), InFileName, _tcslen(InFileName)-4);
    OutFileName[_tcslen(InFileName)-3] = NULL;
    _tcsncat_s(OutFileName, (sizeof(OutFileName)/sizeof(OutFileName[0])), TEXT("_48KHz.wav"), sizeof(TCHAR)*11);
    
    //dump the extra data
    pWFX->cbSize = 0;

    if(MMSYSERR_NOERROR != OpenWriteWaveFile(OutFileName, pWFX->cbSize))
    {
        return 0;
    }

    // init updscale vars
    upscale_engine_init();

    // Update conversion factors for current sample rate.
    upscale_engine_set_current_sample_rate(pWFX->nSamplesPerSec, pWFX->nChannels);


    g_OutFileSize = 0;
    InSamples = inBufferSize/sizeof(INT16);
    pInBuff = (INT16*)pinBuffer;

    pOutBuff = tmpBuf;
    nDst = sizeof(tmpBuf) / sizeof(Int16);


    // Simulate data showing up and being re-sampled
    while(0 < InSamples)
    {
        //randomize samples to read to simulate real operation
#ifdef RANDOMIZE_INPUT_SIZE
            InSamplesRead = rand()%MAX_SAMPLE_TO_READ;
#else
            InSamplesRead = MAX_SAMPLE_TO_READ;
#endif

        if((int)InSamplesRead > InSamples)
        {
            //don't go over end of file
            InSamplesRead = InSamples;
        }

        // Process Simulated paket
        if (upscale_engine_resample_copy(pInBuff,
                                         &InSamplesRead,
                                         pOutBuff,
                                         &nDst))
        {
            //overrun happened
            _tprintf(TEXT("Overrun \r\n"));
        }
        
        // Copy the converted samples to the internal buffer.
        File_Write(pOutBuff, nDst);

        // reset available space in buffer
        nDst = sizeof(tmpBuf) / sizeof(Int16);

        //Advance the simulated input buffer
        pInBuff += InSamplesRead;
        InSamples -= InSamplesRead;
    }

    // update to scaled Freq
	pWFX->nSamplesPerSec		= MATCH_FREQ;
	pWFX->nAvgBytesPerSec		= pWFX->nSamplesPerSec * pWFX->nBlockAlign;


    if(MMSYSERR_NOERROR != CloseWriteWaveFile(pWFX, g_OutFileSize))
    {
        return 0;
    }

    return 1;

}

