package frc.apps.paytoplay

import android.app.Application
import com.atruedev.kmpnfc.adapter.KmpNfc

class MyApp : Application() {
    override fun onCreate() {
        super.onCreate()
        // Explicitly initialize kmp-nfc so ActivityTracker is registered.
        // The library has an AndroidX Startup auto-initializer, but calling
        // init() here guarantees it works regardless of Startup ordering.
        KmpNfc.init(this)
    }
}