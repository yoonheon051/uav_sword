plugins {
    id("com.android.application")
    id("org.jetbrains.kotlin.android")
}

android {
    namespace = "com.igeo.wristsword"
    compileSdk = 34

    defaultConfig {
        applicationId = "com.igeo.wristsword"
        minSdk = 30          // Wear OS 3 (Galaxy Watch4 이상)
        targetSdk = 34
        versionCode = 1
        versionName = "0.1"
    }

    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_17
        targetCompatibility = JavaVersion.VERSION_17
    }
    kotlinOptions {
        jvmTarget = "17"
    }
}

// 외부 라이브러리 없이 안드로이드 기본 API만 사용합니다.
