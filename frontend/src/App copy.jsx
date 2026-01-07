import React, { useState, useRef, Suspense, useEffect } from "react";
import { Canvas, useFrame, useLoader } from "@react-three/fiber";
import { OrbitControls, Stage, Float, Stars, Sparkles, ContactShadows, Environment, Lightformer } from "@react-three/drei";
import { TextureLoader } from "three";
import * as THREE from "three";
import Admin from "./Admin";
import Popup from "./Popup"; // 👈 추가
// ✅ Firebase 라이브러리
import { initializeApp } from "firebase/app";
import { getDatabase, ref, onValue, update } from "firebase/database";
import PaymentModal from "./PaymentModal"; // 👈 추가
import OrderSuccessModal from "./OrderSuccessModal"; // 👈 추가

// ------------------------------------------------------------------
// ⚠️ [필수] Firebase 설정
// ------------------------------------------------------------------
const firebaseConfig = {
  databaseURL: "https://rokey-ad6ec-default-rtdb.asia-southeast1.firebasedatabase.app",
  // apiKey: "AIzaSy...", 
  // projectId: "rokey-ad6ec",
};

// Firebase 초기화
const app = initializeApp(firebaseConfig);
const db = getDatabase(app);

const BACKEND_URL = "http://127.0.0.1:8000"; // 백엔드 주소 상수

// [설정] 기본 도안 데이터
const BASIC_DESIGNS = [
  { id: 3, src: "/basic_print/3.png", label: "개" },
  { id: 4, src: "/basic_print/4.png", label: "가나디" },
];

// =================================================================
// ❄️ [NEW] 웹사이트 배경 눈 내리는 효과 컴포넌트
// =================================================================
function WebSnowOverlay() {
  return (
    <div className="fixed inset-0 pointer-events-none z-0 overflow-hidden" aria-hidden="true">
      {/* 눈송이 생성 */}
      {[...Array(60)].map((_, i) => (
        <div
          key={i}
          className="absolute bg-white rounded-full opacity-60"
          style={{
            top: `${Math.random() * -20}%`,
            left: `${Math.random() * 100}%`,
            width: `${Math.random() * 4 + 2}px`,
            height: `${Math.random() * 4 + 2}px`,
            animation: `fall ${Math.random() * 10 + 10}s linear infinite`,
            animationDelay: `${Math.random() * -10}s`,
            boxShadow: "0 0 5px rgba(255, 255, 255, 0.8)"
          }}
        />
      ))}
      <style>{`
        @keyframes fall {
          0% { transform: translateY(-10vh) translateX(0px) rotate(0deg); opacity: 0.8; }
          100% { transform: translateY(110vh) translateX(20px) rotate(360deg); opacity: 0; }
        }
      `}</style>
    </div>
  );
}


//로그인관련
function LoginModal({ onClose, onLogin }) {
  const [password, setPassword] = useState("");
  const [error, setError] = useState("");

  const handleSubmit = () => {
    if (password === "1234") { // 비밀번호 확인 로직
      onLogin();
    } else {
      setError("⛔ 비밀번호가 일치하지 않습니다.");
      setPassword(""); 
    }
  };

  return (
    <div className="fixed inset-0 bg-black/70 flex items-center justify-center z-[9999] backdrop-blur-sm">
      <div className="bg-white p-8 rounded-2xl shadow-2xl max-w-sm w-full">
        {/* ... 디자인 코드 ... */}
        <h2 className="text-2xl font-bold text-gray-800 text-center mb-4">관리자 접근</h2>
        <input 
          type="password" 
          value={password}
          onChange={(e) => setPassword(e.target.value)}
          onKeyDown={(e) => e.key === 'Enter' && handleSubmit()}
          placeholder="비밀번호 (1234)"
          className="w-full px-4 py-3 border rounded-xl mb-4 text-center"
          autoFocus
        />
        {error && <p className="text-red-500 text-xs text-center mb-2">{error}</p>}
        <div className="grid grid-cols-2 gap-3">
          <button onClick={onClose} className="py-3 bg-gray-100 rounded-xl font-bold">취소</button>
          <button onClick={handleSubmit} className="py-3 bg-blue-600 text-white rounded-xl font-bold">로그인</button>
        </div>
      </div>
    </div>
  );
}

// =================================================================
// 🎨 [NEW] AI 이미지 생성기 컴포넌트 (패널 내장형)
// =================================================================
function AIGenerator({ cakeSize, onUploadSuccess, onLog, isActive }) {
    const [prompt, setPrompt] = useState("");
    const [isGenerating, setIsGenerating] = useState(false);
    const [generatedPreview, setGeneratedPreview] = useState(null);
    const [generatedPaths, setGeneratedPaths] = useState(null);
    const [step, setStep] = useState("input"); // input -> generated -> done

    const handleGenerate = async () => {
        if (!prompt) return;
        setIsGenerating(true);
        setGeneratedPreview(null);
        onLog(`🤖 AI에게 그림 요청 중: "${prompt}"...`);

        try {
            const res = await fetch(`${BACKEND_URL}/api/generate_image`, {
                method: "POST",
                headers: { "Content-Type": "application/json" },
                body: JSON.stringify({ prompt: prompt }),
            });
            const data = await res.json();

            if (data.status === "success") {
                const imageUrl = `${BACKEND_URL}${data.image_url}`;
                setGeneratedPreview(imageUrl);
                setGeneratedPaths(data.paths);
                setStep("generated");
                onLog("✨ AI가 그림을 완성했습니다! (자동 적용됨)");
                
                // 생성 즉시 자동 적용 (미리보기 및 DB 저장)
                handleApply(data.paths, imageUrl);
            } else {
                alert("생성 실패: " + data.message);
                onLog("❌ 생성 실패");
            }
        } catch (err) {
            console.error(err);
            alert("서버 연결 실패");
        } finally {
            setIsGenerating(false);
        }
    };

    // 자동 저장을 위해 분리된 함수
    const handleApply = async (paths, imageUrl) => {
        if (!paths) return;
        
        // UI 상태 업데이트를 위해 상위 컴포넌트에 알림 (이미지 즉시 표시)
        onUploadSuccess({ design_id: "temp_ai_id" }, imageUrl); 
        
        onLog("💾 AI 도안 데이터 저장 중...");

        try {
            const payload = {
                paths: paths,
                size: cakeSize
            };

            const res = await fetch(`${BACKEND_URL}/api/save_custom_paths`, {
                method: "POST",
                headers: { "Content-Type": "application/json" },
                body: JSON.stringify(payload)
            });
            const data = await res.json();

            if (data.status === "success") {
                setStep("done");
                onLog(`🎉 저장 완료! Design ID: ${data.design_id}`);
                // 최종 ID로 업데이트
                onUploadSuccess({ design_id: data.design_id }, imageUrl);
            } else {
                alert("저장 실패: " + data.message);
            }
        } catch (err) {
            console.error(err);
            alert("DB 저장 중 오류 발생");
        }
    };

    return (
        <div className={`p-4 rounded-xl border transition-all ${isActive ? 'bg-purple-50 border-purple-500 ring-1 ring-purple-500' : 'bg-white border-gray-200'}`}>
            <label className="text-xs font-bold text-purple-800 uppercase mb-2 block flex justify-between">
                <span className="flex items-center gap-1">✨ AI 생성형 도안 <span className="bg-purple-600 text-white text-[10px] px-1 rounded">NEW</span></span>
                {isActive && <span className="text-[10px] bg-purple-600 text-white px-2 py-0.5 rounded-full">선택됨</span>}
            </label>

            <div className="flex flex-col gap-3">
                <div className="flex gap-2">
                    <input 
                        type="text" 
                        value={prompt}
                        onChange={(e) => setPrompt(e.target.value)}
                        placeholder="예: 산타 모자를 쓴 고양이"
                        className="flex-1 h-10 px-3 text-sm border border-gray-300 rounded-lg focus:outline-none focus:border-purple-500"
                        onKeyDown={(e) => e.key === 'Enter' && handleGenerate()}
                    />
                    <button 
                        onClick={handleGenerate} 
                        disabled={isGenerating || !prompt}
                        className="h-10 px-3 bg-purple-600 text-white text-xs font-bold rounded-lg hover:bg-purple-500 disabled:bg-gray-300 transition shadow-sm"
                    >
                        {isGenerating ? "..." : "생성"}
                    </button>
                </div>

                {/* 미리보기 영역 (생성된 경우에만 표시) */}
                {(generatedPreview || isGenerating) && (
                    <div className="relative w-full aspect-video bg-gray-100 rounded-lg overflow-hidden border border-gray-200 group">
                         {isGenerating ? (
                             <div className="absolute inset-0 flex flex-col items-center justify-center text-gray-400 gap-2">
                                 <div className="w-6 h-6 border-2 border-purple-200 border-t-purple-600 rounded-full animate-spin"></div>
                                 <span className="text-[10px]">그리는 중...</span>
                             </div>
                         ) : (
                            <>
                                <img src={generatedPreview} alt="AI Result" className="w-full h-full object-contain" />
                                {step === "done" && (
                                    <div className="absolute top-2 right-2 bg-green-500 text-white text-[10px] font-bold px-2 py-1 rounded-full shadow">
                                        적용됨
                                    </div>
                                )}
                            </>
                         )}
                    </div>
                )}
            </div>
        </div>
    );
}

// =================================================================
// 🍰 [Section 1] Landing Page Components (크리스마스 에디션 🎄)
// =================================================================

/**
 * 🎅 3D 산타할아버지 컴포넌트
 */
function SantaClaus({ radius = 3.5, speed = 0.5 }) {
  const group = useRef();
  
  useFrame((state) => {
    const t = state.clock.getElapsedTime() * speed;
    if (group.current) {
        group.current.position.x = Math.sin(t) * radius;
        group.current.position.z = Math.cos(t) * radius;
        group.current.rotation.y = t + Math.PI / 2;
        group.current.position.y = Math.abs(Math.sin(t * 8)) * 0.1;
        group.current.rotation.z = Math.sin(t * 8) * 0.1;
    }
  });

  return (
    <group ref={group} scale={[0.8, 0.8, 0.8]}>
       <mesh position={[-0.2, 0.3, 0]}><cylinderGeometry args={[0.12, 0.12, 0.6]} /><meshStandardMaterial color="#D32F2F" /></mesh>
       <mesh position={[0.2, 0.3, 0]}><cylinderGeometry args={[0.12, 0.12, 0.6]} /><meshStandardMaterial color="#D32F2F" /></mesh>
       <mesh position={[-0.2, 0.05, 0.1]} castShadow><boxGeometry args={[0.15, 0.15, 0.25]} /><meshStandardMaterial color="#1a1a1a" /></mesh>
       <mesh position={[0.2, 0.05, 0.1]} castShadow><boxGeometry args={[0.15, 0.15, 0.25]} /><meshStandardMaterial color="#1a1a1a" /></mesh>
       <mesh position={[0, 0.9, 0]} castShadow><sphereGeometry args={[0.48, 32, 32]} /><meshStandardMaterial color="#D32F2F" /></mesh>
       <mesh position={[0, 0.9, 0.4]} scale={[0.2, 1, 0.1]}><sphereGeometry args={[0.2]} /><meshStandardMaterial color="#FFFFFF" /></mesh>
       <mesh position={[0, 0.8, 0]}><cylinderGeometry args={[0.49, 0.49, 0.1, 32]} /><meshStandardMaterial color="#1a1a1a" /></mesh>
       <mesh position={[0, 0.8, 0.48]}><boxGeometry args={[0.2, 0.15, 0.05]} /><meshStandardMaterial color="#FFD700" metalness={0.8} /></mesh>
       <mesh position={[0, 1.45, 0]}><sphereGeometry args={[0.3]} /><meshStandardMaterial color="#FFCCBC" /></mesh>
       <mesh position={[0, 1.32, 0.15]}><sphereGeometry args={[0.28, 32, 32]} /><meshStandardMaterial color="#FFFFFF" /></mesh>
       <mesh position={[-0.12, 1.52, 0.25]}><sphereGeometry args={[0.035]} /><meshStandardMaterial color="#000" /></mesh>
       <mesh position={[0.12, 1.52, 0.25]}><sphereGeometry args={[0.035]} /><meshStandardMaterial color="#000" /></mesh>
       <mesh position={[0, 1.46, 0.29]}><sphereGeometry args={[0.06]} /><meshStandardMaterial color="#FF7043" /></mesh>
       <mesh position={[-0.22, 1.42, 0.22]}><sphereGeometry args={[0.06]} /><meshStandardMaterial color="#FFAB91" transparent opacity={0.8} /></mesh>
       <mesh position={[0.22, 1.42, 0.22]}><sphereGeometry args={[0.06]} /><meshStandardMaterial color="#FFAB91" transparent opacity={0.8} /></mesh>
       <group position={[0, 1.7, 0]} rotation={[-0.2, 0, 0]}>
          <mesh position={[0, -0.1, 0]}><torusGeometry args={[0.3, 0.08, 16, 32]} rotation={[Math.PI/2, 0, 0]} /><meshStandardMaterial color="#FFFFFF" /></mesh>
          <mesh position={[0, 0.3, 0]}><coneGeometry args={[0.28, 0.8, 32]} /><meshStandardMaterial color="#D32F2F" /></mesh>
          <mesh position={[0, 0.7, 0]}><sphereGeometry args={[0.09]} /><meshStandardMaterial color="#FFFFFF" /></mesh>
       </group>
       <mesh position={[0.45, 1.0, 0]} rotation={[0, 0, -0.5]}><cylinderGeometry args={[0.1, 0.1, 0.6]} /><meshStandardMaterial color="#D32F2F" /></mesh>
       <mesh position={[-0.45, 1.0, 0]} rotation={[0, 0, 0.5]}><cylinderGeometry args={[0.1, 0.1, 0.6]} /><meshStandardMaterial color="#D32F2F" /></mesh>
       <mesh position={[0.6, 0.75, 0]}><sphereGeometry args={[0.12]} /><meshStandardMaterial color="#1B5E20" /></mesh>
       <mesh position={[-0.6, 0.75, 0]}><sphereGeometry args={[0.12]} /><meshStandardMaterial color="#1B5E20" /></mesh>
       <mesh position={[0, 1.0, -0.55]} rotation={[0, 0, -0.2]} castShadow><sphereGeometry args={[0.45, 32, 32]} /><meshStandardMaterial color="#8D6E63" roughness={1} /></mesh>
    </group>
  );
}

/**
 * ☃️ 3D 눈사람 컴포넌트
 */
function Snowman({ position, scale = 1, rotation = [0, 0, 0] }) {
  return (
    <group position={position} scale={[scale, scale, scale]} rotation={rotation}>
      <mesh position={[0, 0.5, 0]} castShadow><sphereGeometry args={[0.6, 32, 32]} /><meshStandardMaterial color="#ffffff" roughness={0.5} /></mesh>
      <mesh position={[0, 1.3, 0]} castShadow><sphereGeometry args={[0.45, 32, 32]} /><meshStandardMaterial color="#ffffff" roughness={0.5} /></mesh>
      <mesh position={[0, 1.95, 0]} castShadow><sphereGeometry args={[0.3, 32, 32]} /><meshStandardMaterial color="#ffffff" roughness={0.5} /></mesh>
      <mesh position={[-0.1, 2.05, 0.25]}><sphereGeometry args={[0.03, 16, 16]} /><meshStandardMaterial color="#000" /></mesh>
      <mesh position={[0.1, 2.05, 0.25]}><sphereGeometry args={[0.03, 16, 16]} /><meshStandardMaterial color="#000" /></mesh>
      <mesh position={[0, 2.0, 0.3]} rotation={[Math.PI / 2, 0, 0]}><coneGeometry args={[0.04, 0.2, 16]} /><meshStandardMaterial color="#ff6b00" /></mesh>
      <mesh position={[0, 1.4, 0.4]}><sphereGeometry args={[0.04, 16, 16]} /><meshStandardMaterial color="#333" /></mesh>
      <mesh position={[0, 1.2, 0.43]}><sphereGeometry args={[0.04, 16, 16]} /><meshStandardMaterial color="#333" /></mesh>
      <mesh position={[0.4, 1.4, 0]} rotation={[0, 0, -0.5]}><cylinderGeometry args={[0.02, 0.02, 0.6]} /><meshStandardMaterial color="#5d4037" /></mesh>
      <mesh position={[-0.4, 1.4, 0]} rotation={[0, 0, 0.5]}><cylinderGeometry args={[0.02, 0.02, 0.6]} /><meshStandardMaterial color="#5d4037" /></mesh>
      <group position={[0, 2.2, 0]} rotation={[-0.1, 0, 0.1]} scale={[0.8, 0.8, 0.8]}>
         <mesh position={[0, 0, 0]}><torusGeometry args={[0.35, 0.1, 16, 32]} rotation={[Math.PI/2, 0, 0]}/><meshStandardMaterial color="#FFFFFF" /></mesh>
         <mesh position={[0, 0.4, 0]}><coneGeometry args={[0.32, 0.8, 32]} /><meshStandardMaterial color="#D32F2F" /></mesh>
         <mesh position={[0, 0.8, 0]}><sphereGeometry args={[0.1]} /><meshStandardMaterial color="#FFFFFF" /></mesh>
      </group>
    </group>
  );
}

/**
 * 🏠 산타 마을 집 컴포넌트
 */
function SantaHouse({ position, rotation = [0, 0, 0], scale = 1 }) {
  return (
    <group position={position} rotation={rotation} scale={[scale, scale, scale]}>
      <mesh position={[0, 1, 0]} castShadow><boxGeometry args={[2, 2, 2]} /><meshStandardMaterial color="#b71c1c" /></mesh>
      <mesh position={[0, 2.5, 0]} rotation={[0, Math.PI / 4, 0]}><coneGeometry args={[2, 1.5, 4]} /><meshStandardMaterial color="#eceff1" /></mesh>
      <mesh position={[0, 0.6, 1.01]}><planeGeometry args={[0.6, 1.2]} /><meshStandardMaterial color="#5d4037" /></mesh>
      <mesh position={[-0.6, 1.2, 1.01]}><planeGeometry args={[0.5, 0.5]} /><meshStandardMaterial color="#ffeb3b" emissive="#ffeb3b" emissiveIntensity={0.8} /></mesh>
      <mesh position={[0.6, 1.2, 1.01]}><planeGeometry args={[0.5, 0.5]} /><meshStandardMaterial color="#ffeb3b" emissive="#ffeb3b" emissiveIntensity={0.8} /></mesh>
      <mesh position={[0.6, 2.5, 0.5]}><boxGeometry args={[0.4, 0.8, 0.4]} /><meshStandardMaterial color="#5d4037" /></mesh>
      <mesh position={[0.6, 2.91, 0.5]}><boxGeometry args={[0.45, 0.1, 0.45]} /><meshStandardMaterial color="#fff" /></mesh>
    </group>
  );
}

/**
 * 🎄 3D 크리스마스 트리 컴포넌트
 */
function ChristmasTree({ position, scale = 1 }) {
  return (
    <group position={position} scale={[scale, scale, scale]}>
      <mesh position={[0, 0.8, 0]} castShadow><coneGeometry args={[0.5, 1.0, 16]} /><meshStandardMaterial color="#1B5E20" roughness={0.8} /></mesh>
      <mesh position={[0, 1.5, 0]} castShadow><coneGeometry args={[0.4, 0.9, 16]} /><meshStandardMaterial color="#2E7D32" roughness={0.8} /></mesh>
      <mesh position={[0, 2.1, 0]} castShadow><coneGeometry args={[0.3, 0.7, 16]} /><meshStandardMaterial color="#388E3C" roughness={0.8} /></mesh>
      <mesh position={[0, 0.25, 0]} castShadow><cylinderGeometry args={[0.12, 0.12, 0.6, 16]} /><meshStandardMaterial color="#5D4037" roughness={1} /></mesh>
      <mesh position={[0, 2.55, 0]}><dodecahedronGeometry args={[0.12, 0]} /><meshStandardMaterial color="#FFD700" emissive="#FFD700" emissiveIntensity={0.6} /></mesh>
      <mesh position={[0.25, 0.9, 0.2]} castShadow><sphereGeometry args={[0.06, 16, 16]} /><meshStandardMaterial color="#D32F2F" metalness={0.6} roughness={0.2} /></mesh>
      <mesh position={[-0.2, 1.6, 0.15]} castShadow><sphereGeometry args={[0.05, 16, 16]} /><meshStandardMaterial color="#FBC02D" metalness={0.6} roughness={0.2} /></mesh>
       <mesh position={[0.1, 0.6, -0.3]} castShadow><sphereGeometry args={[0.06, 16, 16]} /><meshStandardMaterial color="#1976D2" metalness={0.6} roughness={0.2} /></mesh>
    </group>
  );
}

/**
 * 🍰 3D 고급 1단 케이크 컴포넌트
 */
function FloatingCake() {
  const groupRef = useRef();
  useFrame((state, delta) => {
    if (groupRef.current) {
      groupRef.current.rotation.y += delta * 0.15;
    }
  });

  return (
    <Float speed={1.5} rotationIntensity={0.2} floatIntensity={0.5} floatingRange={[-0.1, 0.1]}>
      <group ref={groupRef} position={[0, -0.8, 0]} rotation={[0.1, 0, 0]}>
        <mesh receiveShadow castShadow position={[0, 0.6, 0]}>
          <cylinderGeometry args={[1.4, 1.4, 1.2, 64]} />
          <meshStandardMaterial color="#FAF9F6" roughness={0.25} metalness={0.1} />
        </mesh>
        <mesh position={[0, 1.21, 0]}><cylinderGeometry args={[1.42, 1.42, 0.05, 64]} /><meshStandardMaterial color="#E53935" roughness={0.1} metalness={0.2} /></mesh>
        <mesh position={[0, 1.18, 0]}><torusGeometry args={[1.4, 0.06, 16, 100]} /><meshStandardMaterial color="#E53935" roughness={0.1} metalness={0.2} /></mesh>
        <mesh position={[0, 1.4, 0]} castShadow><sphereGeometry args={[0.35, 32, 32]} /><meshStandardMaterial color="#C62828" roughness={0.1} metalness={0.3} /></mesh>
        {[...Array(6)].map((_, i) => {
            const angle = (i / 6) * Math.PI * 2;
            return <mesh key={i} position={[Math.cos(angle)*1.1, 1.25, Math.sin(angle)*1.1]}><sphereGeometry args={[0.08, 16, 16]} /><meshStandardMaterial color="#FFD700" metalness={0.8} roughness={0.1} /></mesh>
        })}
        <Sparkles count={80} scale={[2, 0.5, 2]} size={4} speed={0.4} opacity={1} color="#FFD700" position={[0, 1.35, 0]} />
        <Sparkles count={20} scale={2.5} size={2} speed={0.3} opacity={0.4} color="#FFF" position={[0, 1, 0]} />
      </group>
    </Float>
  );
}

/**
 * ❄️ 크리스마스 분위기 배경 (눈 + 트리)
 */
function ChristmasBackground() {
  return (
    <>
      <Environment preset="night" />
      <Sparkles count={500} scale={[20, 20, 10]} size={3} speed={0.5} opacity={0.8} color="#FFFFFF" position={[0, 5, 0]} />
      <Stars radius={100} depth={50} count={2000} factor={4} saturation={0} fade speed={1} />
      
      <ChristmasTree position={[-4, -2.5, -4]} scale={1.8} />
      <ChristmasTree position={[4, -2.5, -3]} scale={1.5} />
      <ChristmasTree position={[-2.5, -2.5, -6]} scale={2.2} />
      <ChristmasTree position={[2.5, -2.5, -5]} scale={1.2} />

      <mesh position={[0, -2.6, 0]} rotation={[-Math.PI / 2, 0, 0]} receiveShadow>
         <circleGeometry args={[20, 64]} />
         <meshStandardMaterial color="#ECEFF1" roughness={1} metalness={0} />
      </mesh>

      <pointLight position={[0, 2, 0]} intensity={0.5} color="#FFD700" />
      <spotLight position={[5, 5, 5]} angle={0.3} penumbra={1} intensity={2} castShadow color="#FF8A80" />
      <spotLight position={[-5, 5, 5]} angle={0.3} penumbra={1} intensity={2} castShadow color="#80DEEA" />
      <ContactShadows position={[0, -2.5, 0]} opacity={0.4} scale={20} blur={2.5} far={4.5} color="#000000" />
    </>
  );
}

/**
 * 🚀 Landing Page 컴포넌트 (크리스마스 에디션)
 */
function LandingPage({ onStart, onAdmin }) {
  const [isHovered, setIsHovered] = useState(false);

  return (
    <div className="relative w-full h-screen bg-[#05081c] overflow-hidden font-sans selection:bg-rose-500 selection:text-white">
      <button 
        onClick={onAdmin}
        className="absolute top-6 right-6 z-[100] flex items-center gap-2 px-5 py-2.5 rounded-full bg-white/90 hover:bg-white text-slate-600 font-bold shadow-sm backdrop-blur-sm border border-slate-200 transition-all duration-300 hover:shadow-md active:scale-95 group"
        title="관리자 페이지 이동"
      >
        <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" fill="currentColor" className="w-4 h-4 text-slate-400 group-hover:text-pink-500 transition-colors">
          <path fillRule="evenodd" d="M12 1.5a5.25 5.25 0 00-5.25 5.25v3a3 3 0 00-3 3v6.75a3 3 0 003 3h10.5a3 3 0 003-3v-6.75a3 3 0 00-3-3v-3c0-2.9-2.35-5.25-5.25-5.25zm3.75 8.25v-3a3.75 3.75 0 10-7.5 0v3h7.5z" clipRule="evenodd" />
        </svg>
        <span className="text-sm">관리자 페이지</span>
      </button>
      
      <div className="absolute inset-0 z-0">
        <Canvas camera={{ position: [0, 0, 8], fov: 40 }}>
          <color attach="background" args={['#05081c']} />
          <ambientLight intensity={0.3} />
          <Suspense fallback={null}>
            <FloatingCake />
            <ChristmasBackground />
          </Suspense>
        </Canvas>
      </div>
      
      <div className="relative z-10 w-full h-full flex flex-col items-center justify-between py-12 px-6">
        <header className="w-full max-w-7xl flex justify-between items-center opacity-90">
            <div className="flex items-center gap-3">
                <div className="w-8 h-8 rounded-full border border-rose-400/30 flex items-center justify-center bg-rose-900/20 backdrop-blur-sm">
                    <div className="w-1.5 h-1.5 bg-rose-400 rounded-full animate-pulse shadow-[0_0_10px_#fb7185]" />
                </div>
                <span className="text-white font-light tracking-[0.2em] text-sm drop-shadow-lg">ROKEY SYSTEM</span>
            </div>
            <div className="hidden md:flex gap-8 text-xs font-light text-rose-100/70 tracking-widest uppercase">
                <span>Christmas Edition</span>
                <span>Technology</span>
                <span>Experience</span>
            </div>
        </header>
        <main className="flex flex-col items-center text-center space-y-8 mt-[-40px]">
            <div className="animate-in fade-in slide-in-from-top-4 duration-1000 delay-100">
                <span className="px-5 py-2 rounded-full border border-rose-200/20 bg-rose-900/10 backdrop-blur-md text-rose-200 text-xs font-medium tracking-widest uppercase shadow-[0_0_15px_rgba(244,63,94,0.2)]">
                    🎄 Holiday Special Bakery
                </span>
            </div>
            <div className="space-y-2">
                <h1 className="animate-in zoom-in duration-1000 delay-200 text-5xl md:text-7xl lg:text-8xl font-thin text-white tracking-tighter leading-none drop-shadow-2xl">
                    <span className="block font-medium bg-clip-text text-transparent bg-gradient-to-br from-white via-rose-50 to-rose-200">Merry</span>
                    <span className="block text-rose-100/50 font-thin italic -mt-2 md:-mt-4">Christmas</span>
                </h1>
            </div>
            <p className="animate-in fade-in slide-in-from-bottom-4 duration-1000 delay-300 max-w-lg text-rose-100/70 font-light leading-relaxed text-sm md:text-base drop-shadow-md">
                이번 크리스마스에는 로봇 셰프가 만드는<br/>
                가장 특별한 <strong>커스텀 케이크</strong>를 선물하세요.
            </p>
            <div className="animate-in fade-in slide-in-from-bottom-8 duration-1000 delay-500 pt-8">
                <button
                    onClick={onStart}
                    onMouseEnter={() => setIsHovered(true)}
                    onMouseLeave={() => setIsHovered(false)}
                    className="group relative px-12 py-4 bg-transparent overflow-hidden rounded-none"
                >
                    <div className="absolute inset-0 border border-rose-200/30 group-hover:border-rose-200/60 transition-colors duration-300 shadow-[0_0_20px_rgba(244,63,94,0.1)]" />
                    <div className={`absolute inset-0 bg-rose-500/10 transition-transform duration-500 origin-left ${isHovered ? 'scale-x-100' : 'scale-x-0'}`} />
                    <span className="relative z-10 text-white font-light tracking-[0.3em] text-sm group-hover:text-white transition-colors drop-shadow-lg">ORDER NOW</span>
                    <div className="absolute top-0 left-0 w-2 h-2 border-t border-l border-rose-200/60" />
                    <div className="absolute bottom-0 right-0 w-2 h-2 border-b border-r border-rose-200/60" />
                </button>
            </div>
        </main>
        <footer className="w-full max-w-7xl flex justify-between items-end text-[10px] text-rose-200/40 font-mono tracking-wider uppercase">
            <div className="hidden md:block">Powered by Doosan Robotics<br/>Holiday Season v2.5</div>
            <div className="text-right">System Status: Online<br/>Merry Christmas ❄️</div>
        </footer>
      </div>
    </div>
  );
}

// =================================================================
// 🔧 [Section 2] Existing Modals & Helpers
// =================================================================

function CompletionModal({ onClose }) {
  return (
    <div className="fixed inset-0 bg-black/60 flex items-center justify-center z-[9999] backdrop-blur-sm animate-in fade-in zoom-in duration-300">
      <div className="bg-white p-8 rounded-3xl shadow-2xl max-w-sm w-full text-center border-4 border-green-400 relative overflow-hidden">
        <div className="absolute top-0 left-0 w-full h-full bg-gradient-to-br from-green-50 to-blue-50 -z-10"></div>
        <div className="w-24 h-24 bg-green-100 rounded-full flex items-center justify-center mx-auto mb-6 shadow-inner animate-bounce">
          <span className="text-5xl">🎂</span>
        </div>
        <h2 className="text-2xl font-black text-gray-800 mb-2">제작이 완성되었습니다!</h2>
        <p className="text-gray-500 font-medium mb-8">
          맛있는 케이크가 준비되었습니다.<br/>
          픽업대에서 확인해주세요.
        </p>
        <button 
          onClick={onClose}
          className="w-full py-4 bg-green-500 hover:bg-green-400 text-white font-bold rounded-xl text-lg shadow-lg transition transform hover:scale-105"
        >
          확인 (처음으로)
        </button>
      </div>
    </div>
  );
}

function EmergencyModal({ onResume, onReset }) {
  return (
    <div className="fixed inset-0 bg-red-900/80 flex items-center justify-center z-[9999] backdrop-blur-md animate-pulse-slow">
      <div className="bg-white p-8 rounded-3xl shadow-2xl max-w-md w-full text-center border-4 border-red-500">
        <div className="w-20 h-20 bg-red-100 rounded-full flex items-center justify-center mx-auto mb-6">
          <span className="text-4xl">⚠️</span>
        </div>
        <h2 className="text-2xl font-black text-red-600 mb-2">케이크 상태를 확인하세요!</h2>
        <p className="text-gray-600 font-bold mb-8">
          로봇이 일시 정지되었습니다.<br/>
          케이크와 로봇 상태를 확인 후 작업을 결정해주세요.
        </p>
        <div className="grid grid-cols-2 gap-4">
          <button 
            onClick={onResume}
            className="py-4 bg-blue-600 hover:bg-blue-500 text-white font-bold rounded-xl text-lg shadow-lg transition transform hover:scale-105"
          >
            ▶️ 재개 (Resume)
          </button>
          <button 
            onClick={onReset}
            className="py-4 bg-gray-200 hover:bg-gray-300 text-gray-700 font-bold rounded-xl text-lg shadow-inner transition"
          >
            🔄 초기화 (Reset)
          </button>
        </div>
      </div>
    </div>
  );
}

function PhotoUploader({ cakeSize, onUploadSuccess, onLog, isActive }) {
    const [selectedFile, setSelectedFile] = useState(null);
    const [preview, setPreview] = useState(null);
    const [analyzedPaths, setAnalyzedPaths] = useState(null);
    const [isUploading, setIsUploading] = useState(false);
    const [step, setStep] = useState("select");

    const handleFileChange = (e) => {
        const file = e.target.files[0];
        if (file) {
            setSelectedFile(file);
            setPreview(URL.createObjectURL(file));
            setStep("select");
            setAnalyzedPaths(null);
        }
    };

    const handleAnalyze = async () => {
        if (!selectedFile) return;
        setIsUploading(true);
        onLog("🔍 도안 분석 중...");

        const formData = new FormData();
        formData.append("file", selectedFile);

        try {
            const res = await fetch(`${BACKEND_URL}/api/analyze_image`, {
                method: "POST",
                body: formData,
            });
            const data = await res.json();

            if (data.status === "success") {
                setAnalyzedPaths(data.paths);
                setStep("analyze");
                onLog(`✅ 분석 완료! (획 수: ${data.paths.length}) - 적용을 눌러주세요.`);
            } else {
                alert("분석 실패: " + data.message);
            }
        } catch (err) {
            console.error(err);
            alert("서버 연결 실패");
        } finally {
            setIsUploading(false);
        }
    };

    const handleApply = async () => {
        if (!analyzedPaths) return;
        setIsUploading(true);
        onLog("💾 도안 데이터 DB 저장 중...");

        try {
            const payload = {
                paths: analyzedPaths,
                size: cakeSize
            };

            const res = await fetch(`${BACKEND_URL}/api/save_custom_paths`, {
                method: "POST",
                headers: { "Content-Type": "application/json" },
                body: JSON.stringify(payload)
            });
            const data = await res.json();

            if (data.status === "success") {
                setStep("done");
                onLog(`🎉 적용 완료! Design ID: ${data.design_id}`);
                onUploadSuccess({ design_id: data.design_id }, preview);
            } else {
                alert("저장 실패: " + data.message);
            }
        } catch (err) {
            console.error(err);
            alert("DB 저장 중 오류 발생");
        } finally {
            setIsUploading(false);
        }
    };

    return (
        <div className={`p-4 rounded-xl border transition-all ${isActive ? 'bg-indigo-50 border-indigo-500 ring-1 ring-indigo-500' : 'bg-white border-gray-200'}`}>
            <label className="text-xs font-bold text-indigo-800 uppercase mb-2 block flex justify-between">
                <span>📸 커스텀 도안 (나만의 사진)</span>
                {isActive && <span className="text-[10px] bg-indigo-600 text-white px-2 py-0.5 rounded-full">선택됨</span>}
            </label>

            <div className="flex flex-col gap-3">
                <div className="flex gap-2">
                    <label className="flex-1 cursor-pointer">
                        <input type="file" accept="image/*" onChange={handleFileChange} className="hidden" />
                        <div className="w-full h-10 bg-white border border-dashed border-gray-300 rounded-lg flex items-center justify-center text-xs text-gray-500 hover:bg-gray-50 hover:border-indigo-400 transition">
                            {selectedFile ? selectedFile.name : "+ 이미지 업로드"}
                        </div>
                    </label>
                    {selectedFile && step === "select" && (
                        <button 
                            onClick={handleAnalyze} 
                            disabled={isUploading}
                            className="h-10 px-3 bg-indigo-600 text-white text-xs font-bold rounded-lg hover:bg-indigo-500 disabled:bg-gray-400 transition"
                        >
                            {isUploading ? "..." : "분석"}
                        </button>
                    )}
                </div>

                {preview && (
                    <div className="relative w-full aspect-video bg-gray-100 rounded-lg overflow-hidden border border-gray-200 group">
                        <img src={preview} alt="Preview" className="w-full h-full object-contain" />
                        {step === "analyze" && (
                            <div className="absolute inset-0 bg-black/40 flex flex-col items-center justify-center gap-2 backdrop-blur-[1px]">
                                <p className="text-white text-xs font-bold">분석이 완료되었습니다!</p>
                                <button 
                                    onClick={handleApply}
                                    disabled={isUploading}
                                    className="px-4 py-2 bg-green-500 text-white text-sm font-bold rounded-full shadow-lg hover:bg-green-400 hover:scale-105 transition"
                                >
                                    {isUploading ? "저장 중..." : "✅ 이대로 적용하기"}
                                </button>
                            </div>
                        )}
                        {step === "done" && (
                            <div className="absolute top-2 right-2 bg-green-500 text-white text-[10px] font-bold px-2 py-1 rounded-full shadow">
                                적용됨
                            </div>
                        )}
                    </div>
                )}
            </div>
        </div>
    );
}

// -----------------------------------------------------------------
// 🍼 [NEW] 시럽통 (Syrup Bottle) 컴포넌트
// -----------------------------------------------------------------
function SyrupBottle({ position, color, isActive, label }) {
  const groupRef = useRef();
  
  // 선택 시 위로 톡 튀어오르는 애니메이션
  useFrame((state, delta) => {
    if (groupRef.current) {
      const targetY = isActive ? position[1] + 0.5 : position[1];
      // 부드러운 이동 (Lerp)
      groupRef.current.position.y = THREE.MathUtils.lerp(groupRef.current.position.y, targetY, delta * 10);
    }
  });

  return (
    <group ref={groupRef} position={position}>
      {/* 병 몸통 */}
      <mesh position={[0, 0.4, 0]} castShadow>
        <cylinderGeometry args={[0.2, 0.25, 0.8, 32]} />
        <meshStandardMaterial color={color} roughness={0.3} metalness={0.1} />
      </mesh>
      {/* 라벨 (띠) */}
      <mesh position={[0, 0.4, 0]}>
        <cylinderGeometry args={[0.21, 0.26, 0.4, 32]} />
        <meshStandardMaterial color="#ffffff" side={THREE.DoubleSide} />
      </mesh>
      {/* 병 목 */}
      <mesh position={[0, 0.85, 0]}>
        <cylinderGeometry args={[0.1, 0.15, 0.2, 32]} />
        <meshStandardMaterial color={color} />
      </mesh>
      {/* 뚜껑 (입구) */}
      <mesh position={[0, 1.0, 0]}>
        <cylinderGeometry args={[0.08, 0.08, 0.15, 32]} />
        <meshStandardMaterial color="#333" />
      </mesh>
      {/* 라벨 텍스트 효과 (간단히 박스로 표현) */}
      <mesh position={[0, 0.4, 0.22]}>
         <boxGeometry args={[0.2, 0.15, 0.01]} />
         <meshStandardMaterial color={color} />
      </mesh>
    </group>
  );
}

function RobotStatusModal({ onClose }) {
  const [statusData, setStatusData] = useState(null);

  useEffect(() => {
    const statusRef = ref(db, 'robots/dsr01/snapshot');
    const unsubscribe = onValue(statusRef, (snapshot) => {
      setStatusData(snapshot.val());
    });
    return () => unsubscribe();
  }, []);

  if (!statusData) {
    return (
      <div className="fixed inset-0 bg-black/50 flex items-center justify-center z-50 backdrop-blur-sm">
        <div className="bg-white p-8 rounded-2xl shadow-2xl flex flex-col items-center">
          <div className="animate-spin rounded-full h-10 w-10 border-b-2 border-blue-600 mb-4"></div>
          <p className="font-bold text-gray-600">로봇 데이터 수신 중...</p>
          <button onClick={onClose} className="mt-6 text-sm text-gray-400 hover:text-gray-600">닫기</button>
        </div>
      </div>
    );
  }

  const { joint_states, tcp_pose, robot_state } = statusData;

  return (
    <div className="fixed inset-0 bg-black/60 flex items-center justify-center z-50 backdrop-blur-sm p-4">
      <div className="bg-white w-full max-w-2xl rounded-2xl shadow-2xl overflow-hidden flex flex-col max-h-[90vh]">
        <div className="bg-slate-800 p-4 flex justify-between items-center">
          <h2 className="text-white font-bold text-lg flex items-center gap-2">
            🤖 Real-time Robot Status
            <span className="text-[10px] bg-green-500 text-white px-2 py-0.5 rounded-full animate-pulse">LIVE</span>
          </h2>
          <button onClick={onClose} className="text-slate-400 hover:text-white text-2xl">&times;</button>
        </div>

        <div className="p-6 overflow-y-auto bg-slate-50 flex-1 grid grid-cols-1 md:grid-cols-2 gap-6">
          <div className="col-span-1 md:col-span-2 bg-white p-4 rounded-xl border border-gray-200 shadow-sm flex justify-between items-center">
             <div>
                <p className="text-xs text-gray-400 font-bold uppercase">Robot State</p>
                <p className="text-xl font-black text-blue-600">{robot_state?.text || "Unknown"}</p>
             </div>
             <div className="text-right">
                <p className="text-xs text-gray-400 font-bold uppercase">Timestamp</p>
                <p className="text-sm font-mono text-gray-600">
                    {statusData.timestamp ? new Date(statusData.timestamp * 1000).toLocaleTimeString() : "-"}
                </p>
             </div>
          </div>
          <div className="bg-white p-4 rounded-xl border border-gray-200 shadow-sm">
            <h3 className="text-sm font-bold text-gray-700 mb-3 border-b pb-2">🦾 Joint Angles (deg)</h3>
            <div className="grid grid-cols-2 gap-y-2 text-sm">
                {[1, 2, 3, 4, 5, 6].map(i => (
                    <div key={i} className="flex justify-between px-2">
                        <span className="font-semibold text-gray-400">J{i}</span>
                        <span className="font-mono font-bold text-gray-700">
                            {joint_states?.[`joint_${i}`]?.toFixed(2) ?? 0}
                        </span>
                    </div>
                ))}
            </div>
          </div>
          <div className="bg-white p-4 rounded-xl border border-gray-200 shadow-sm">
            <h3 className="text-sm font-bold text-gray-700 mb-3 border-b pb-2">📍 TCP Pose</h3>
            <div className="space-y-3">
                <div className="grid grid-cols-3 gap-2 text-center">
                    {['x', 'y', 'z'].map(axis => (
                        <div key={axis} className="bg-blue-50 rounded p-1">
                            <div className="text-[10px] text-blue-400 font-bold uppercase">{axis}</div>
                            <div className="font-mono text-sm font-bold">{tcp_pose?.[axis]?.toFixed(1) ?? 0}</div>
                        </div>
                    ))}
                </div>
                <div className="grid grid-cols-3 gap-2 text-center">
                    {['roll', 'pitch', 'yaw'].map(axis => (
                        <div key={axis} className="bg-orange-50 rounded p-1">
                            <div className="text-[10px] text-orange-400 font-bold capitalize">{axis}</div>
                            <div className="font-mono text-sm font-bold">{tcp_pose?.[axis]?.toFixed(1) ?? 0}</div>
                        </div>
                    ))}
                </div>
            </div>
          </div>
        </div>
        <div className="p-4 bg-gray-100 border-t text-center">
            <button onClick={onClose} className="w-full py-3 bg-slate-800 text-white font-bold rounded-xl hover:bg-slate-700 transition">닫기</button>
        </div>
      </div>
    </div>
  );
}

function CakeDrawing({ url }) {
  const texture = useLoader(TextureLoader, url);
  return (
    <mesh position={[0, 1.02, 0]} rotation={[-Math.PI / 2, 0, 0]}>
      <circleGeometry args={[1.4, 64]} /> 
      <meshStandardMaterial map={texture} transparent opacity={0.9} roughness={0.5} />
    </mesh>
  );
}

function RobotArm({ isWorking }) {
  const baseRef = useRef();
  const shoulderRef = useRef();
  const elbowRef = useRef();
  const wristRef = useRef();
  const toolRef = useRef();

  useFrame((state) => {
    if (isWorking) {
      const time = state.clock.getElapsedTime();
      if (baseRef.current) baseRef.current.rotation.y = Math.sin(time) * 0.4; 
      if (shoulderRef.current) shoulderRef.current.rotation.x = Math.sin(time * 2) * 0.05 + 0.2; 
      if (elbowRef.current) elbowRef.current.rotation.x = Math.cos(time * 2) * 0.05 + 1.5;
      if (wristRef.current) wristRef.current.rotation.x = 1.2 - Math.sin(time * 2) * 0.1; 
      if (toolRef.current) toolRef.current.rotation.y += 0.2;
    } else {
      if (baseRef.current) baseRef.current.rotation.y = 0;
      if (shoulderRef.current) shoulderRef.current.rotation.x = -0.3; 
      if (elbowRef.current) elbowRef.current.rotation.x = 2.0; 
      if (wristRef.current) wristRef.current.rotation.x = -1.5; 
    }
  });

  const DOOSAN_WHITE = "#f7f7f7";
  const DOOSAN_NAVY = "#002a5c"; 

  return (
    <group position={[3.8, 0, 0]} rotation={[0, -Math.PI / 2, 0]}>
      <mesh position={[0, 0.15, 0]} castShadow>
        <cylinderGeometry args={[0.7, 0.8, 0.3, 32]} />
        <meshStandardMaterial color={DOOSAN_WHITE} roughness={0.3} />
      </mesh>
      <group ref={baseRef}>
        <mesh position={[0, 0.5, 0]} castShadow>
          <cylinderGeometry args={[0.5, 0.5, 0.7, 32]} />
          <meshStandardMaterial color={DOOSAN_WHITE} roughness={0.3} />
        </mesh>
        <mesh position={[0, 0.86, 0]}>
          <cylinderGeometry args={[0.51, 0.51, 0.05, 32]} />
          <meshStandardMaterial color={DOOSAN_NAVY} metalness={0.6} />
        </mesh>
        <group position={[0, 1.1, 0]} rotation={[0, 0, 0]}>
            <mesh rotation={[Math.PI/2, 0, 0]} castShadow>
                <cylinderGeometry args={[0.5, 0.5, 0.6, 32]} />
                <meshStandardMaterial color={DOOSAN_WHITE} roughness={0.3} />
            </mesh>
            <mesh rotation={[Math.PI/2, 0, 0]} position={[0, 0, 0.31]}>
                <cylinderGeometry args={[0.4, 0.4, 0.05, 32]} />
                <meshStandardMaterial color={DOOSAN_NAVY} />
            </mesh>
            <group ref={shoulderRef}>
                 <mesh position={[0, 1.2, 0]} castShadow>
                    <cylinderGeometry args={[0.35, 0.4, 2.4, 32]} />
                    <meshStandardMaterial color={DOOSAN_WHITE} roughness={0.3} />
                 </mesh>
                 <group position={[0, 2.4, 0]} ref={elbowRef}>
                      <mesh rotation={[Math.PI/2, 0, 0]} castShadow>
                        <cylinderGeometry args={[0.4, 0.4, 0.5, 32]} />
                        <meshStandardMaterial color={DOOSAN_WHITE} roughness={0.3} />
                      </mesh>
                      <mesh rotation={[Math.PI/2, 0, 0]} position={[0, 0, 0.26]}>
                        <cylinderGeometry args={[0.3, 0.3, 0.05, 32]} />
                        <meshStandardMaterial color={DOOSAN_NAVY} />
                      </mesh>
                      <mesh position={[0, 1.1, 0]} castShadow>
                         <cylinderGeometry args={[0.25, 0.3, 2.2, 32]} />
                         <meshStandardMaterial color={DOOSAN_WHITE} roughness={0.3} />
                      </mesh>
                      <group position={[0, 2.2, 0]} ref={wristRef}>
                          <mesh rotation={[Math.PI/2, 0, 0]} castShadow>
                             <cylinderGeometry args={[0.28, 0.28, 0.4, 32]} />
                             <meshStandardMaterial color={DOOSAN_WHITE} />
                          </mesh>
                          
                          {/* 🎄 Robot Party Hat (Added here on the wrist/head) */}
                          <group position={[0, 0.5, 0.2]} rotation={[0.5, 0, 0]}>
                              <mesh position={[0, 0.3, 0]} castShadow>
                                  <coneGeometry args={[0.15, 0.5, 32]} />
                                  <meshStandardMaterial color="#D32F2F" />
                              </mesh>
                              <mesh position={[0, 0.05, 0]}>
                                  <torusGeometry args={[0.15, 0.05, 16, 32]} />
                                  <meshStandardMaterial color="#FFF" />
                              </mesh>
                              <mesh position={[0, 0.55, 0]}>
                                  <sphereGeometry args={[0.05]} />
                                  <meshStandardMaterial color="#FFF" />
                              </mesh>
                          </group>

                          <group position={[0, 0.4, 0]} rotation={[0, 0, 0]}>
                              <mesh position={[0, 0, 0]}>
                                 <cylinderGeometry args={[0.25, 0.25, 0.3, 32]} />
                                 <meshStandardMaterial color={DOOSAN_NAVY} metalness={0.7} />
                              </mesh>
                              <group position={[0, 0.3, 0]} ref={toolRef}>
                                  <mesh castShadow>
                                    <cylinderGeometry args={[0.08, 0.15, 0.6, 32]} />
                                    <meshStandardMaterial color="#c0c0c0" metalness={0.9} roughness={0.1} />
                                  </mesh>
                                  {isWorking && (
                                    <mesh position={[0, 0.4, 0]}>
                                      <sphereGeometry args={[0.08, 8, 8]} />
                                      <meshBasicMaterial color="pink" />
                                    </mesh>
                                  )}
                              </group>
                          </group>
                      </group>
                 </group>
            </group>
        </group>
      </group>
    </group>
  );
}

function CakeModel({ size, design, syrup, powder, toppings, drawingUrl }) {
  const scale = size === "1호" ? 1 : 1.4;

  // 1. { strawberry: 2, mango: 1 } -> ['strawberry', 'strawberry', 'mango'] 형태로 변환
  const toppingList = Object.entries(toppings || {}).flatMap(([type, count]) => 
    Array(count).fill(type)
  );
  
  const totalCount = toppingList.length;
  const syrupColors = {
    'choco_syrup': '#3E2723',
    'strawberry_syrup': '#D81B60'
  };

  const powderColors = {
    'sugar_powder': '#ffffff',
    'choco_powder': '#5D4037'
  };

  const toppingColors = {
    'strawberry': '#FF1744',
    'mango': '#FFC107',
    'blueberry': '#3F51B5'
  };

  return (
    <group scale={[scale, scale, scale]}>
      {/* 1. 빵 */}
      <mesh position={[0, 0.5, 0]} castShadow>
        <cylinderGeometry args={[1.5, 1.5, 1, 32]} />
        <meshStandardMaterial color={design === 'chocolate' ? "#8D6E63" : "#fefefe"} />
      </mesh>
      
      {/* 2. 중간 크림 */}
      <mesh position={[0, 0.5, 0]}>
        <cylinderGeometry args={[1.52, 1.52, 0.1, 32]} />
        <meshStandardMaterial color={design === 'chocolate' ? "#5D4037" : (design === 'banana' ? "#FFF176" : "#ffcccb")} />
      </mesh>

      {/* 3. 파우더 */}
      {powder && (
        <mesh position={[0, 1.01, 0]} rotation={[-Math.PI/2, 0, 0]}>
           <circleGeometry args={[1.45, 32]} />
           <meshStandardMaterial 
              color={powder === 'sugar_powder' ? '#ffffff' : '#5D4037'} 
              transparent 
              opacity={0.6} 
              roughness={1} 
           />
        </mesh>
      )}

      {/* 5. 도안 렌더링 */}
      {drawingUrl && <CakeDrawing url={drawingUrl} />}

     {/* 6. 토핑 렌더링 */}
      {totalCount > 0 && (
        <group position={[0, 1.05, 0]}>
          {toppingList.map((toppingType, i) => {
            const angle = (i / totalCount) * Math.PI * 2; 
            const r = 1.25;
            const xPos = Math.cos(angle) * r;
            const zPos = Math.sin(angle) * r;

            return (
              <group key={i} position={[xPos, 0, zPos]} rotation={[0, -angle, 0]}>
                {toppingType === 'strawberry' && (
                  <mesh castShadow>
                    <coneGeometry args={[0.12, 0.2, 16]} />
                    <meshStandardMaterial color="#FF1744" />
                  </mesh>
                )}
                {toppingType === 'mango' && (
                  <group position={[0, 0.05, 0]} rotation={[Math.random() * 0.2, Math.random() * Math.PI, 0]}>
                    {/* 1. 메인 과육 조각 (부드러운 큐브 형태) */}
                    <mesh castShadow>
                      <boxGeometry args={[0.22, 0.18, 0.22]} />
                      <meshStandardMaterial
                        color="#FFC107"           // 진한 망고색
                        roughness={0.15}          // 매끄러운 표면
                        metalness={0.1}           // 살짝의 반사광
                        emissive="#FF8F00"        // 과육 안쪽에서 우러나오는 색감
                        emissiveIntensity={0.2}
                      />
                    </mesh>

                    {/* 2. 과즙 코팅 (얇은 투명 층) - 신선한 느낌 추가 */}
                    <mesh position={[0, 0.01, 0]}>
                      <boxGeometry args={[0.24, 0.16, 0.24]} />
                      <meshStandardMaterial
                        color="#FFFFFF"
                        transparent
                        opacity={0.2}
                        roughness={0}
                      />
                    </mesh>

                    {/* 3. 상단 하이라이트 (시럽 광택 표현) */}
                    <mesh position={[0.06, 0.09, 0.06]}>
                      <sphereGeometry args={[0.035, 8, 8]} />
                      <meshStandardMaterial color="#FFFFFF" transparent opacity={0.6} />
                    </mesh>
                  </group>
                )}
                {toppingType === 'blueberry' && (
                  <mesh castShadow>
                    <sphereGeometry args={[0.1, 16, 16]} />
                    <meshStandardMaterial color="#3F51B5" />
                  </mesh>
                )}
              </group>
            );
          })}
        </group>
      )}
    </group>
  );
}

// --- 옵션 데이터 ---
const SYRUP_OPTIONS = [
    { id: 'choco_syrup', label: '초코 시럽', color: '#5D4037' },
    { id: 'strawberry_syrup', label: '딸기 시럽', color: '#D81B60' },
];

const POWDER_OPTIONS = [
    { id: 'sugar_powder', label: '슈가 파우더', color: '#FFFFFF', borderColor: '#ddd' },
    { id: 'choco_powder', label: '초코 파우더', color: '#5D4037', borderColor: '#5D4037' },
];


// =================================================================
// 🎤 [NEW] 음성 AI 상담원 컴포넌트 (플로팅 버튼)
// =================================================================
function VoiceCounselor({ onLog,onOrderConfirm,currentContext, onUpdateContext }) {
  const [isRecording, setIsRecording] = useState(false);
  const [isProcessing, setIsProcessing] = useState(false);
  const mediaRecorderRef = useRef(null);
  const audioChunksRef = useRef([]);

  // 마이크 녹음 시작
  const startRecording = async () => {
    try {
      const stream = await navigator.mediaDevices.getUserMedia({ audio: true });
      mediaRecorderRef.current = new MediaRecorder(stream);
      audioChunksRef.current = [];

      mediaRecorderRef.current.ondataavailable = (event) => {
        if (event.data.size > 0) audioChunksRef.current.push(event.data);
      };

      mediaRecorderRef.current.onstop = async () => {
        const audioBlob = new Blob(audioChunksRef.current, { type: 'audio/wav' });
        sendAudioToServer(audioBlob);
      };

      mediaRecorderRef.current.start(1000);
      setIsRecording(true);
      onLog("🎤 말씀을 듣고 있습니다... (버튼을 다시 누르면 전송)");
    } catch (err) {
      alert("마이크 권한이 필요합니다.");
      console.error(err);
    }
  };

  // 녹음 중지
  const stopRecording = () => {
    if (mediaRecorderRef.current && isRecording) {
      setTimeout(() => {
        if (mediaRecorderRef.current.state === "recording") {
          mediaRecorderRef.current.stop();
        }
        setIsRecording(false);
        setIsProcessing(true); // 서버 응답 대기 상태 시작
      }, 100);

    }
  };

  const sendAudioToServer = async (blob) => {
    const formData = new FormData();
    formData.append('file', blob, 'user_voice.wav');
    
    // ⭐️ [핵심] 현재까지 알고 있는 주문 맥락을 같이 보냅니다.
    // 백엔드의 current_context: str = Body(None) 부분을 타겟팅합니다.
    formData.append('current_context', JSON.stringify(currentContext));

    try {
      const res = await fetch(`${BACKEND_URL}/api/voice_counselor`, { 
        method: 'POST', 
        body: formData 
      });
      const data = await res.json();

      if (data.status === "error") throw new Error(data.message);

      // 1. AI 목소리 재생
      const audioSrc = `data:audio/mp3;base64,${data.audio_base64}`;
      const audio = new Audio(audioSrc);
      onLog("🍰 사장님이 대답하는 중...");
      await audio.play();

      // 2. ⭐️ 백엔드에서 새롭게 추출된 정보가 있다면 리액트 상태 업데이트
      // (백엔드 응답에 extracted_fields가 포함되도록 백엔드 수정이 필요합니다)
      if (data.order_data && Object.keys(data.order_data).length > 0) {
          onUpdateContext(data.order_data);
      }

      audio.onended = () => {
        setIsProcessing(false);
        // 3. 모든 정보가 완료되어 confirm_order가 내려오면 모달 팝업
        if (data.action === "confirm_order") {
          onOrderConfirm(data.order_data); 
        }
      };
    } catch (e) {
      onLog("❌ 상담 오류");
      setIsProcessing(false);
    }
  };

  return (
    <div className="fixed bottom-8 right-8 z-[100] flex flex-col items-end gap-3">
      {/* 도움말 풍선 */}
      <div className={`bg-white px-4 py-2 rounded-2xl shadow-xl border border-rose-100 text-xs font-bold text-rose-500 transition-all duration-300 transform ${isRecording || isProcessing ? 'translate-y-0 opacity-100' : 'translate-y-2 opacity-0'}`}>
        {isRecording ? "말씀이 끝나면 버튼을 눌러주세요!" : isProcessing ? "답변 준비 중..." : ""}
      </div>

      {/* 메인 마이크 버튼 */}
      <button
        onClick={isRecording ? stopRecording : startRecording}
        disabled={isProcessing}
        className={`w-16 h-16 rounded-full shadow-2xl flex items-center justify-center transition-all duration-300 transform active:scale-95 ${
          isRecording 
            ? 'bg-rose-500 animate-pulse scale-110' 
            : isProcessing 
              ? 'bg-gray-400 cursor-not-allowed' 
              : 'bg-gradient-to-br from-rose-400 to-rose-600 hover:rotate-12 hover:scale-105'
        }`}
      >
        {isProcessing ? (
          <div className="w-6 h-6 border-2 border-white/30 border-t-white rounded-full animate-spin"></div>
        ) : (
          <span className="text-3xl">{isRecording ? "⏹️" : "🎤"}</span>
        )}
        
        {/* 장식용 링 애니메이션 */}
        {isRecording && (
          <div className="absolute inset-0 rounded-full bg-rose-500 animate-ping opacity-20"></div>
        )}
      </button>
      
      <p className="text-[10px] font-black text-white bg-rose-600/50 backdrop-blur-sm px-2 py-1 rounded uppercase tracking-tighter shadow-sm">
        AI Counselor
      </p>
    </div>
  );
}

function OrderCheckModal({ data, onConfirm, onCancel }) {
  if (!data) return null;
  const safeToppings = Array.isArray(data.toppings) ? data.toppings : [];
  const toppingsDisplay = safeToppings.length > 0 
    ? safeToppings.join(', ') 
    : "없음";
  return (
    <div className="fixed inset-0 bg-black/70 flex items-center justify-center z-[9999] backdrop-blur-sm animate-in fade-in zoom-in duration-300">
      <div className="bg-white w-full max-w-sm rounded-3xl p-6 shadow-2xl border-4 border-rose-200">
        <h2 className="text-2xl font-black text-slate-800 mb-2 text-center">🧾 주문서 확인</h2>
        <p className="text-xs text-gray-400 text-center mb-6">AI 사장님이 정리한 내용이 맞나요?</p>
        <div className="bg-rose-50 p-4 rounded-xl space-y-3 mb-6 font-bold text-slate-700">
          <div className="flex justify-between"><span>사이즈:</span><span className="text-rose-600">{data.size}</span></div>
          <div className="flex justify-between"><span>도안:</span><span>{data.design_keyword || "기본"}</span></div>
          <div className="flex justify-between"><span>시럽:</span><span>{data.syrup === 'none' ? '없음' : data.syrup}</span></div>
          <div className="flex justify-between"><span>파우더:</span><span>{data.powder === 'none' ? '없음' : data.powder}</span></div>
          <div className="flex justify-between items-start">
            <span className="shrink-0">토핑:</span>
            <span className="text-right text-sm text-blue-600 break-keep">
                {toppingsDisplay} 
                {data.topping_count > 0 && <span className="text-xs text-gray-500 ml-1">({data.topping_count}개)</span>}
            </span>
          </div>
        </div>
        <div className="grid grid-cols-2 gap-3">
          <button onClick={onCancel} className="py-3 bg-gray-200 text-gray-600 font-bold rounded-xl">취소</button>
          <button onClick={onConfirm} className="py-3 bg-rose-500 text-white font-bold rounded-xl shadow-lg transition transform hover:scale-105">예 (주문)</button>
        </div>
      </div>
    </div>
  );
}

// =================================================================
// 📡 [NEW] 로봇 거리 감지 센서 제어기 (영상 X, 소리 제어용)
// =================================================================
function RobotSensorWidget() {
  const [triggerValue, setTriggerValue] = useState(300); // 초기값
  const [isExpanded, setIsExpanded] = useState(false);   // 기본적으로 접어둠

  // 거리 조절 핸들러
  const updateTrigger = async (change) => {
    const newValue = triggerValue + change;
    if (newValue < 50) return; 

    try {
      setTriggerValue(newValue);
      // 백엔드로 설정값 전송
      await fetch(`${BACKEND_URL}/api/update_trigger`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ value: newValue })
      });
    } catch (err) {
      console.error("Trigger Update Failed:", err);
    }
  };

  return (
    <div className="fixed bottom-8 left-8 z-[100] flex flex-col items-start gap-2 animate-in slide-in-from-left duration-700">
      
      {/* 1. 센서 상태 표시줄 (클릭하면 열림) */}
      <div 
        className="bg-white/90 backdrop-blur border border-slate-200 text-slate-700 px-4 py-3 rounded-2xl shadow-xl flex items-center gap-3 cursor-pointer hover:bg-white transition hover:scale-105 active:scale-95"
        onClick={() => setIsExpanded(!isExpanded)}
      >
        {/* 작동 중 표시 (초록불) */}
        <span className="relative flex h-3 w-3">
          <span className="animate-ping absolute inline-flex h-full w-full rounded-full bg-green-400 opacity-75"></span>
          <span className="relative inline-flex rounded-full h-3 w-3 bg-green-500"></span>
        </span>
        
        <div className="flex flex-col">
            <span className="text-xs font-extrabold uppercase tracking-wider text-slate-800">Approach Sensor</span>
            <span className="text-[10px] text-slate-500 font-medium">Auto-Welcome Active</span>
        </div>

        {/* 현재 설정값 표시 */}
        <div className="ml-2 px-2 py-1 bg-slate-100 rounded text-xs font-mono font-bold text-blue-600">
            {triggerValue}px
        </div>
      </div>

      {/* 2. 감도 조절 패널 (열렸을 때만 보임) */}
      {isExpanded && (
        <div className="bg-white/90 backdrop-blur p-4 rounded-2xl shadow-2xl border border-slate-100 w-full animate-in fade-in slide-in-from-bottom-2">
          <div className="flex justify-between items-center mb-2">
            <span className="text-xs font-bold text-slate-500">감지 민감도 조절</span>
          </div>
          
          <div className="flex items-center gap-2 justify-between bg-slate-50 p-1 rounded-xl">
            <button 
              onClick={(e) => { e.stopPropagation(); updateTrigger(-10); }}
              className="w-8 h-8 flex items-center justify-center bg-white border border-slate-200 rounded-lg shadow-sm hover:bg-rose-50 hover:text-rose-500 hover:border-rose-200 active:scale-90 transition font-bold text-lg text-slate-400"
            >
              -
            </button>
            
            <div className="flex flex-col items-center">
                <span className="text-sm font-black text-slate-700">{triggerValue}</span>
                <span className="text-[9px] text-slate-400">THRESHOLD</span>
            </div>

            <button 
              onClick={(e) => { e.stopPropagation(); updateTrigger(+10); }}
              className="w-8 h-8 flex items-center justify-center bg-white border border-slate-200 rounded-lg shadow-sm hover:bg-blue-50 hover:text-blue-500 hover:border-blue-200 active:scale-90 transition font-bold text-lg text-slate-400"
            >
              +
            </button>
          </div>
          <p className="text-[10px] text-slate-400 mt-2 text-center break-keep leading-tight">
            값이 클수록 더 멀리서도 인식합니다.<br/>(권장: 200~400)
          </p>
        </div>
      )}
    </div>
  );
}

// ==================================================================================
// 🚀 메인 App 컴포넌트
// ==================================================================================
export default function App() {
  const [isStarted, setIsStarted] = useState(false); // ✅ 랜딩 페이지 표시 여부 상태
  const [view, setView] = useState('landing');
  const [showLoginModal, setShowLoginModal] = useState(false);
  const [toppingCount, setToppingCount] = useState(6);
  const [selectedToppings, setSelectedToppings] = useState({}); // { strawberry: 2, mango: 3 } 식의 객체
  const currentTotalToppings = Object.values(selectedToppings).reduce((sum, count) => sum + count, 0);
  const isToppingComplete = currentTotalToppings === 8; // 정확히 8개인지 확인
  const [showPickupModal, setShowPickupModal] = useState(false); // 팝업 표시 여부
  const [showPaymentModal, setShowPaymentModal] = useState(false); // 👈 결제 모달 상태 추가
  const [tempPickupData, setTempPickupData] = useState(null);      // 👈 픽업 데이터 임시 저장용
  const [showOrderSuccessModal, setShowOrderSuccessModal] = useState(false); // 주문 성공 모달 표시 여부
  const [confirmedOrderInfo, setConfirmedOrderInfo] = useState({ id: null, waitTime: 15 }); // 확정된 주문 정보

    // 토핑 선택/해제 핸들러
  const handleToppingToggle = (toppingId) => {
    setSelectedToppings(prev => {
      const newToppings = { ...prev };
      if (newToppings[toppingId]) {
        delete newToppings[toppingId]; // 이미 있으면 삭제
      } else {
        const currentTotal = Object.values(newToppings).reduce((a, b) => a + b, 0);
        if (currentTotal < 8) {
          newToppings[toppingId] = 1; // 없으면 1개로 시작
        } else {
          alert("전체 토핑은 최대 8개까지 가능합니다.");
        }
      }
      return newToppings;
    });
  };

  // 특정 토핑의 개수 조절 핸들러
  const handleIndividualToppingCount = (toppingId, count) => {
    setSelectedToppings(prev => {
      const newToppings = { ...prev };
      const otherTotal = Object.entries(newToppings)
        .filter(([id]) => id !== toppingId)
        .reduce((sum, [_, val]) => sum + val, 0);

      if (otherTotal + count <= 8) {
        newToppings[toppingId] = count;
      } else {
        alert("전체 토핑 합계가 8개를 초과할 수 없습니다.");
        newToppings[toppingId] = 8 - otherTotal; // 최대치로 강제 설정
      }
      return newToppings;
    });
  };
    

  const handleLoginSuccess = () => {
    setShowLoginModal(false); 
    setView('admin');         
  };

  const TOPPING_OPTIONS = [
    { id: 'strawberry', label: '딸기 토핑', color: '#FF1744' },
    { id: 'mango', label: '망고 토핑', color: '#FFC107' },
    { id: 'blueberry', label: '블루베리 토핑', color: '#3F51B5' },
  ];


  const handleBackToMain = () => {
    setView('landing');
  };
  const [showOrderCheckModal, setShowOrderCheckModal] = useState(false);
  const [autoOrderData, setAutoOrderData] = useState(null);

  const [cakeSize, setCakeSize] = useState("1호");
  const [activeDesign, setActiveDesign] = useState("strawberry");
  const [selectedSyrup, setSelectedSyrup] = useState(null);
  const [selectedPowder, setSelectedPowder] = useState(null);
  
  const [orderType, setOrderType] = useState('basic'); 
  const [selectedBasicDesign, setSelectedBasicDesign] = useState(null); 
  const [drawingUrl, setDrawingUrl] = useState(null);
  const [customDesignId, setCustomDesignId] = useState(null);

  const [isRunning, setIsRunning] = useState(false);
  const [showStatusModal, setShowStatusModal] = useState(false);

  // ⭐️ [신규] 상태 관리: 현재 주문 번호 & 에러 모달
  const [currentOrderId, setCurrentOrderId] = useState(null);
  const [showErrorModal, setShowErrorModal] = useState(false);

  // ⭐️ [추가] 제작 완료 모달 상태
  const [showCompletionModal, setShowCompletionModal] = useState(false);

  const addLog = (msg) => { console.log(`[System Log] ${msg}`); };
  const [aiContext, setAiContext] = useState({
    size: null,
    design_keyword: null,
    syrup: null,
    powder: null,
    topping: null
  });

const handleAdminAccess = () => {
    setShowLoginModal(true); 
  };

  // 사이즈 변경 시 토핑 개수 자동 조절 로직 추가 예시
  useEffect(() => {
    if (cakeSize === "1호") {
      setToppingCount(6);
    } else {
      setToppingCount(8);
    }
  }, [cakeSize]);

  // =================================================================
  // 1️⃣ [추가] 주문 상태 감시 (Status: Done -> 완료 모달)
  // =================================================================
  useEffect(() => {
    // 주문 번호가 없으면 감시하지 않음
    if (!currentOrderId) return;

    const orderStatusRef = ref(db, `orders/${currentOrderId}/status`);

    const unsubscribe = onValue(orderStatusRef, (snapshot) => {
      const status = snapshot.val();
      
      // 상태가 'done'으로 변경되었고, 아직 완료 모달이 안 떴으며, 에러 상황이 아닐 때
      if (status === 'done' && !showCompletionModal && !showErrorModal) {
        addLog(`🎉 주문 #${currentOrderId} 제작 완료!`);
        setIsRunning(false);          // 로봇 동작 애니메이션 중지
        setShowCompletionModal(true); // 완료 모달 띄우기
      }
    });

    return () => unsubscribe();
  }, [currentOrderId, showCompletionModal, showErrorModal]);

  // =================================================================
  // 🚨 [핵심 로직] 로봇 에러 상태 실시간 감시 (useEffect)
  // =================================================================
  useEffect(() => {
    // 로봇 상태 감시 경로
    const statusRef = ref(db, 'robots/dsr01/snapshot');

    const unsubscribe = onValue(statusRef, (snapshot) => {
        const data = snapshot.val();
        if (!data || !data.robot_state) return;

        // robot_state가 객체인지 숫자인지 확인하여 코드 추출
        // (보통 {code: 1, text: "IDLE"} 형태이거나 숫자 1 형태임)
        // ✅ [수정] DB 스냅샷에 따르면 'value' 필드에 상태 코드가 저장되어 있음
        const stateCode = typeof data.robot_state === 'object' ? data.robot_state.value : data.robot_state;

        // ⚠️ 에러 코드 정의 (5, 6, 9, 10)
        const ERROR_CODES = [3,5, 6, 9, 10];

        // 에러 발생 && 현재 작업 중인 주문이 있음 && 아직 모달이 안 떴음 && 완료 모달이 떠있지 않음
        if (ERROR_CODES.includes(stateCode) && currentOrderId && !showErrorModal && !showCompletionModal) {
            console.warn(`🚨 EMERGENCY: Robot Error Code ${stateCode} Detected!`);
            
            // 1. 에러 모달 띄우기
            setShowErrorModal(true);
        }
    });

    return () => unsubscribe();
  }, [currentOrderId, showErrorModal, showCompletionModal]); // 의존성 추가




  
  // -----------------------------------------------------------------
  // 🔘 완료 모달 확인 버튼 핸들러
  // -----------------------------------------------------------------
  const handleCloseCompletion = () => {
    setShowCompletionModal(false); // 모달 닫기
    setCurrentOrderId(null);       // 주문 번호 초기화 (다음 주문 받을 준비)
    addLog("✨ 새로운 주문 준비 완료");
  };

  // -----------------------------------------------------------------
  // 🔘 에러 핸들러: 재개 (Resume)
  // -----------------------------------------------------------------
  const handleResume = async () => {
      console.group("🔍 [DEBUG] 재개(Resume) 버튼 클릭됨");
      
      // 1. 주문 ID 확인
      console.log("1️⃣ 현재 저장된 주문 번호(currentOrderId):", currentOrderId);
      
      if (!currentOrderId) {
          console.error("❌ [ERROR] 주문 번호가 null/undefined 입니다. DB 경로를 찾을 수 없습니다.");
          alert("오류: 현재 작업 중인 주문 번호를 찾을 수 없습니다. (새로고침 하셨나요?)");
          console.groupEnd();
          return;
      }

      // 2. 경로 및 데이터 확인
      const targetPath = `orders/${currentOrderId}`;
      console.log("2️⃣ 업데이트 시도 경로:", targetPath);
      console.log("3️⃣ 보낼 데이터:", { status: 'replaying' }); // 로봇 코드에 따라 'playing' 일수도 있음

      try {
          console.log("⏳ Firebase update 요청 보내는 중...");
          
          // 3. 실제 업데이트 수행
          await update(ref(db, targetPath), { status: 'replaying' });
          
          console.log("✅ [SUCCESS] DB 업데이트 성공!");
          
          // 4. 후처리
          setShowErrorModal(false); // 모달 닫기
          addLog(`✅ 작업 재개됨 (Target: ${targetPath}, Status: replaying)`);
          
      } catch (error) {
          console.error("❌ [FAIL] Firebase Update 실패:", error);
          console.error("   - 에러 코드:", error.code);
          console.error("   - 에러 메시지:", error.message);
          alert("재개 실패(DB 에러): " + error.message);
      } finally {
          console.groupEnd();
      }
  };
  // -----------------------------------------------------------------
  // 🔘 에러 핸들러: 초기화 (Reset)
  // -----------------------------------------------------------------
  const handleReset = async () => {
      if (!currentOrderId) return;
      addLog("🔄 작업 초기화 요청...");

      try {
          // DB 상태를 'done'으로 변경 -> 종료 처리
          await update(ref(db, `orders/${currentOrderId}`), { status: 'reset' });
          
          setShowErrorModal(false); // 모달 닫기
          setIsRunning(false);      // 로컬 실행 상태 끄기
          setCurrentOrderId(null);  // 주문 번호 초기화
          addLog("✅ 작업 강제 종료됨 (Status: done)");
      } catch (error) {
          alert("초기화 실패: " + error.message);
      }
  };

  // --- 기존 핸들러들 ---
  const handleBasicDesignSelect = (id, src) => {
    setOrderType('기본도안'); 
    setSelectedBasicDesign(id);
    setCustomDesignId(null); 
    setDrawingUrl(src); 
    addLog(`🎨 기본 도안 ${id}번 선택됨`);
  };

  const handleCustomUploadSuccess = (result, previewUrl) => {
    setOrderType('커스텀도안'); 
    setSelectedBasicDesign(null);
    if (result.design_id) setCustomDesignId(result.design_id);
    setDrawingUrl(previewUrl);
    addLog(`📸 커스텀 도안 로드 완료 (ID: ${result.design_id})`);
  };
// 기존 handleStart 함수 윗부분에 이 함수를 추가하세요.
const handlePreOrderCheck = () => {
  if (isRunning) return;

  // 토핑 개수 검사 (기존 로직)
  if (currentTotalToppings !== 8) {
    alert(`토핑을 정확히 8개 골라주세요! (현재 ${currentTotalToppings}개)`);
    return;
  }

  // 검사가 통과되면 팝업을 띄웁니다.
  setShowPickupModal(true);
};


// 2. 픽업 방법 선택 완료 -> 결제 모달 열기 (DB 전송 X)
  const handlePickupNext = (pickupData) => {
    setTempPickupData(pickupData); // 데이터 임시 저장
    setShowPickupModal(false);     // 픽업 모달 닫기
    setShowPaymentModal(true);     // 결제 모달 열기 🚀
  };

  // 3. 결제 완료 -> DB 전송 시작 (handleStart 호출)
  const handlePaymentComplete = async () => {
    setShowPaymentModal(false); // 결제 모달 닫기
    
    // 임시 저장해둔 pickupData를 사용하여 실제 주문 전송
    if (tempPickupData) {
        await handleStart(tempPickupData); 
    }
  };

// 기존 handleStart를 아래와 같이 수정하세요.
const handleStart = async (pickupData) => { // 👈 인자 추가됨
  setShowPickupModal(false); // 팝업 닫기
  
  if (isRunning) return;

  // 토핑 ID 매핑 (기존 로직 유지)
  const toppingIdMap = { strawberry: 1, blueberry: 2, mango: 3 };
  const toppingList = Object.entries(selectedToppings).flatMap(([type, count]) =>
    Array(count).fill(toppingIdMap[type])
  );

  // 전송할 데이터 구성
  const orderData = {
    size: cakeSize,
    design: orderType,
    syrup: selectedSyrup || "none",
    powder: selectedPowder || "none",
    toppings: toppingList.length > 0 ? toppingList : [0],
    topping_count: toppingList.length,
    type: orderType,
    design_id: orderType === '기본도안' ? selectedBasicDesign : customDesignId,
    
    // 👇 [NEW] 팝업에서 받은 픽업 정보 추가
    pickup_type: pickupData.type,   // 'onsite' or 'reservation'
    pickup_time: pickupData.time    // 'now' or '2025-01-01T12:00'
  };

  addLog(`⏳ 주문 전송 중 (${pickupData.type === 'onsite' ? '현장' : '예약'}, 토핑: ${toppingList.length}개)...`);
  addLog(`💸 결제 완료! 주문 전송 중...`); // 로그 메시지 변경
  try {
    const response = await fetch(`${BACKEND_URL}/api/order`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(orderData)
    });

    if (!response.ok) throw new Error('서버 응답 오류');
    const result = await response.json();
    
    if (result.order_id) {
        setCurrentOrderId(result.order_id);
        setIsRunning(true);
        const estimatedTime = 10 + Math.floor(Math.random() * 5); // 10~15분 랜덤 예시
          
          setConfirmedOrderInfo({ 
            id: result.order_id, 
            waitTime: estimatedTime 
          });
          setShowOrderSuccessModal(true);
      }
  } catch (error) {
    console.error("Order Failed:", error);
    alert("주문 전송 실패!");
  }
};

  const handleFinalOrder = async () => {
    setShowOrderCheckModal(false);

    // 1. AI가 인식한 한글/영어 토핑 이름을 로봇용 숫자 ID로 변환하는 맵
    const toppingIdMap = { 
        '딸기': 1, 'strawberry': 1, 
        '블루베리': 2, 'blueberry': 2, 
        '망고': 3, 'mango': 3 
    };

    const designIdMap = {
        '개': 3, 'dog': 3, '강아지': 3,
        '가나디': 4, 'cat': 4, '고양이': 4, // 가나디가 4번이라고 가정
        '기본': 3 
    };  

    let finalDesignId = selectedBasicDesign;
    if (!finalDesignId && autoOrderData.design_keyword) {
        const keyword = autoOrderData.design_keyword.trim(); // 공백 제거
        finalDesignId = designIdMap[keyword] || 3; // 매핑 안되면 3번(기본)
    }

    // 2. AI 데이터(autoOrderData.toppings)를 숫자 리스트로 변환
    let finalToppings = [];
    // toppings가 배열인지 확인하고 처리
     if (autoOrderData.toppings && Array.isArray(autoOrderData.toppings)) {
        console.log("📊 [Debug] 원본 토핑 데이터:", autoOrderData.toppings);
        
        // 이미 숫자 배열이면 그대로 사용
        if (autoOrderData.toppings.every(t => typeof t === 'number')) {
            finalToppings = autoOrderData.toppings;
            console.log("✅ [토핑] 숫자 배열로 이미 처리됨:", finalToppings);
        } 
        // 문자열 배열이면 변환
        else {
            finalToppings = autoOrderData.toppings
                .map(t => {
                    const key = String(t).trim().toLowerCase(); 
                    return toppingIdMap[key] || 0;
                })
                .filter(id => id !== 0);
            console.log("🔄 [토핑] 문자열 -> 숫자 변환됨:", finalToppings);
        }
    }

    const orderData = {
      size: autoOrderData.size,
      design: autoOrderData.design_keyword || "기본",
      type: "AI_VOICE_ORDER",
      syrup: (autoOrderData.syrup && autoOrderData.syrup !== 'none') ? autoOrderData.syrup : null,
      powder: (autoOrderData.powder && autoOrderData.powder !== 'none') ? autoOrderData.powder : null,
      // ✨ [수정됨] 변환된 숫자 리스트 전송
      toppings: finalToppings, 
      topping_count: finalToppings.length,

      design_id: finalDesignId
    };
    console.log("📤 [전송 데이터]", JSON.stringify(orderData, null, 2));

     try {
        const res = await fetch(`${BACKEND_URL}/api/order`, { 
            method: 'POST', 
            headers: { 'Content-Type': 'application/json' }, 
            body: JSON.stringify(orderData) 
        });
        
        const result = await res.json();
        
        if (result.order_id) {
            setCurrentOrderId(result.order_id);
            setIsRunning(true);
            alert(`주문 완료! (#${result.order_id})`);
        } else {
            console.error("❌ 주문 실패:", result);
            alert("주문 실패: " + (result.message || "알 수 없는 오류"));
        }
    } catch (err) { 
        console.error("❌ 주문 전송 에러:", err);
        alert("주문 실패");
    }
};

  const handleStop = () => { setIsRunning(false); setCurrentOrderId(null); };

  if (view === 'landing') {
    return (
      <>
        {/* LandingPage에 onAdmin 핸들러 전달 */}
        <LandingPage onStart={() => setView('order')} onAdmin={handleAdminAccess} />
        
        {/* ✅ [수정 4] 조건부로 로그인 모달 렌더링 */}
        {showLoginModal && (
          <LoginModal 
            onClose={() => setShowLoginModal(false)} 
            onLogin={handleLoginSuccess} 
          />
        )}
      </>
    );
  }

  if (view === 'admin') {
    return <Admin onLogout={handleBackToMain} />;
  }
  
  // ⭐️ [변경 포인트] 랜딩 페이지 분기 처리
  if (!isStarted) {
    return <LandingPage onStart={() => setIsStarted(true)} />;
  }

  return (
    <div className="flex flex-col h-screen bg-slate-900 font-sans text-gray-800 animate-in fade-in duration-500 overflow-hidden relative">
      <WebSnowOverlay />
      
      {/* 🚨 긴급 에러 모달 (조건부 렌더링) */}
      {showErrorModal && (
          <EmergencyModal onResume={handleResume} onReset={handleReset} />
      )}

      {/* 🎂 [추가] 제작 완료 모달 */}
      {showCompletionModal && (
          <CompletionModal onClose={handleCloseCompletion} />
      )}

      {showOrderCheckModal && (
        <OrderCheckModal 
          data={autoOrderData} 
          onConfirm={handleFinalOrder} 
          onCancel={() => setShowOrderCheckModal(false)} 
        />
      )}

      {/* 헤더 */}
      <header className="relative z-10 bg-white/90 backdrop-blur-sm px-6 py-4 shadow-sm border-b border-gray-200 flex items-center justify-between">
        <h1 
            className="text-xl font-extrabold text-blue-900 tracking-tight cursor-pointer hover:opacity-80 transition"
            onClick={() => setIsStarted(false)} // 로고 클릭 시 랜딩 페이지로 이동
        >
            🤖 ROKEY ROBOT SYSTEM
        </h1>
        <div className="flex gap-2">
            <div className={`w-3 h-3 rounded-full ${isRunning ? 'bg-green-500 animate-pulse' : 'bg-yellow-500'}`}></div>
            <span className="text-xs font-bold text-gray-600">{isRunning ? `OPERATING (#${currentOrderId})` : 'IDLE'}</span>
        </div>
      </header>

      <main className="relative z-10 flex-1 p-4 grid grid-cols-12 gap-6 h-full overflow-hidden max-w-[1600px] mx-auto w-full">
        {/* 왼쪽 패널 */}
        <section className="col-span-4 lg:col-span-3 bg-white/95 backdrop-blur-md rounded-2xl shadow-lg border border-gray-100 flex flex-col overflow-hidden">
          <div className="p-5 border-b border-gray-100 bg-gray-50/80">
            <h2 className="font-bold text-gray-700 flex items-center gap-2">⚙️ 작업 설정</h2>
          </div>
          <div className="p-5 flex-1 overflow-y-auto flex flex-col gap-6">
            {/* 사이즈 */}
            <div>
                <label className="text-xs font-bold text-gray-400 uppercase mb-2 block">Cake Size</label>
                <div className="grid grid-cols-2 gap-2 bg-gray-100 p-1 rounded-xl">
                    {["1호", "2호"].map(size => (
                        <button key={size} onClick={() => setCakeSize(size)}
                            className={`py-2 text-sm font-bold rounded-lg transition-all ${cakeSize === size ? 'bg-white shadow text-blue-600' : 'text-gray-400 hover:text-gray-600'}`}>
                            {size}
                        </button>
                    ))}
                </div>
            </div>

            {/* ✅ [추가] AI 생성형 도안 컴포넌트 배치 */}
            <AIGenerator 
                cakeSize={cakeSize} 
                onUploadSuccess={handleCustomUploadSuccess} 
                onLog={addLog} 
                isActive={orderType === '커스텀도안'} 
            />

            {/* 기본 도안 선택 */}
            <div className={`p-4 rounded-xl border transition-all ${orderType === '기본도안' ? 'bg-blue-50 border-blue-500 ring-1 ring-blue-500' : 'bg-white border-gray-200'}`}>
                <label className="text-xs font-bold text-blue-800 uppercase mb-2 block flex justify-between">
                    <span>🖌️ 기본 도안 선택</span>
                    {orderType === '기본도안' && <span className="text-[10px] bg-blue-600 text-white px-2 py-0.5 rounded-full">선택됨</span>}
                </label>
                <div className="grid grid-cols-3 gap-2">
                    {BASIC_DESIGNS.map((bd) => (
                        <button key={bd.id} onClick={() => handleBasicDesignSelect(bd.id, bd.src)}
                            className={`aspect-square rounded-lg border overflow-hidden relative group transition-all ${selectedBasicDesign === bd.id ? 'border-blue-500 ring-2 ring-blue-300' : 'border-gray-200 hover:border-blue-300'}`}>
                            <div className="w-full h-full bg-gray-100 flex items-center justify-center text-xs text-gray-400">
                                <img src={bd.src} alt={bd.label} className="w-full h-full object-cover" onError={(e) => {e.target.style.display='none';}} /> 
                                <span className="absolute inset-0 flex items-center justify-center opacity-0 group-hover:opacity-100 bg-black/50 text-white text-xs font-bold transition-opacity">{bd.label}</span>
                            </div>
                        </button>
                    ))}
                </div>
            </div>
            {/* 커스텀 도안 */}
            <PhotoUploader cakeSize={cakeSize} onUploadSuccess={handleCustomUploadSuccess} onLog={addLog} isActive={orderType === '커스텀도안'} />
            
            {/* 시럽 선택 */}
            <div className="bg-blue-50 p-4 rounded-xl border border-blue-100">
               <label className="text-xs font-bold text-blue-800 uppercase mb-3 block">시럽 (Max 1)</label>
               <div className="flex flex-col gap-2">
                   {SYRUP_OPTIONS.map(opt => (
                       <label key={opt.id} className="flex justify-between items-center p-2 cursor-pointer hover:bg-white/50 rounded-lg">
                           <div className="flex items-center gap-2">
                               <div className="w-4 h-4 rounded-full" style={{backgroundColor: opt.color}}></div>
                               <span className="text-sm font-bold text-gray-700">{opt.label}</span>
                           </div>
                           <input type="checkbox" checked={selectedSyrup === opt.id} onChange={() => setSelectedSyrup(selectedSyrup === opt.id ? null : opt.id)} className="w-5 h-5 accent-blue-600 rounded-full" />
                       </label>
                   ))}
               </div>
            </div>

            {/* 파우더 선택 */}
            <div className="bg-gray-50 p-4 rounded-xl border border-gray-200">
               <label className="text-xs font-bold text-gray-600 uppercase mb-3 block">파우더 (Max 1)</label>
               <div className="flex flex-col gap-2">
                   {POWDER_OPTIONS.map(opt => (
                       <label key={opt.id} className="flex justify-between items-center p-2 cursor-pointer hover:bg-white/50 rounded-lg">
                           <div className="flex items-center gap-2">
                               <div className="w-4 h-4 rounded-full border" style={{backgroundColor: opt.color, borderColor: opt.borderColor}}></div>
                               <span className="text-sm font-bold text-gray-700">{opt.label}</span>
                           </div>
                           <input type="checkbox" checked={selectedPowder === opt.id} onChange={() => setSelectedPowder(selectedPowder === opt.id ? null : opt.id)} className="w-5 h-5 accent-gray-600 rounded-full" />
                       </label>
                   ))}
               </div>
            </div>

            {/* 파우더 선택 아래에 추가 */}
            <div className="bg-pink-50 p-5 rounded-2xl border border-pink-100 space-y-4">
              <div className="flex justify-between items-center">
                <label className="text-[10px] font-black text-pink-800 uppercase tracking-widest">
                  🍓 토핑 구성 (반드시 8개 선택)
                </label>
                <span className={`text-xs font-bold px-2 py-0.5 rounded-full ${isToppingComplete ? 'bg-green-500 text-white' : 'bg-pink-200 text-pink-800'}`}>
                  {currentTotalToppings} / 8
                </span>
              </div>
              <div className="flex flex-col gap-3">
                {TOPPING_OPTIONS.map(opt => {
                  const isSelected = !!selectedToppings[opt.id];
                  return (
                    <div key={opt.id} className="space-y-2 bg-white/50 p-3 rounded-xl border border-pink-100">
                      <label className="flex justify-between items-center cursor-pointer group">
                        <div className="flex items-center gap-3">
                          <div className="w-4 h-4 rounded-full" style={{ backgroundColor: opt.color }}></div>
                          <span className="text-sm font-bold text-slate-700">{opt.label}</span>
                        </div>
                        <input
                          type="checkbox"
                          checked={isSelected}
                          onChange={() => handleToppingToggle(opt.id)}
                          className="w-5 h-5 accent-pink-600 rounded-full"
                        />
                      </label>

                      {/* 선택되었을 때만 개수 조절 슬라이더 표시 */}
                      {isSelected && (
                        <div className="pt-2 flex flex-col gap-1 border-t border-pink-100/50">
                          <div className="flex justify-between text-[10px] font-bold text-pink-500">
                            <span>개수</span>
                            <span>{selectedToppings[opt.id]}개</span>
                          </div>
                          <input
                            type="range"
                            min="1"
                            max="8"
                            value={selectedToppings[opt.id]}
                            onChange={(e) => handleIndividualToppingCount(opt.id, parseInt(e.target.value))}
                            className="w-full h-1.5 bg-pink-200 rounded-lg appearance-none cursor-pointer accent-pink-500"
                          />
                        </div>
                      )}
                    </div>
                  );
                })}
              </div>
            </div>

          </div>
        </section>

        {/* 오른쪽 3D 뷰어 */}
        <section className="col-span-8 lg:col-span-9 flex flex-col gap-4">
          <div className="flex-1 bg-gradient-to-b from-gray-900 to-slate-800 rounded-2xl shadow-2xl border border-gray-700 relative overflow-hidden group">
            <Canvas shadows camera={{ position: [5, 5, 8], fov: 40 }}>
              {/* 🎄 배경색: 짙은 밤하늘 */}
              <color attach="background" args={['#101525']} />
              <fog attach="fog" args={['#101525', 10, 30]} />
              
              <ambientLight intensity={0.5} />
              <directionalLight position={[10, 10, 5]} intensity={1.5} castShadow />
              
              {/* ❄️ 눈 내리는 효과 추가 */}
              <Sparkles count={500} scale={[20, 20, 20]} size={4} speed={0.3} opacity={0.8} color="#FFF" />
              <Stars radius={100} depth={50} count={1500} factor={4} fade speed={1} />

              <Suspense fallback={null}>
                <Stage intensity={0.5} environment="city" adjustCamera={false}>

                  <CakeModel
                    size={cakeSize}
                    design={activeDesign}
                    syrup={selectedSyrup}
                    powder={selectedPowder}
                    toppings={selectedToppings}
                    toppingCount={toppingCount}
                    drawingUrl={drawingUrl}
                  />

                    <RobotArm isWorking={isRunning} />
                    
                    {/* === 🎄 크리스마스 데코레이션 === */}
                    {/* 바닥 눈 (넓게) */}
                    <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, -0.05, 0]} receiveShadow>
                        <circleGeometry args={[15, 64]} />
                        <meshStandardMaterial color="#f0f8ff" roughness={1} />
                    </mesh>

                    {/* 트리 */}
                    <ChristmasTree position={[-2.5, 0, -2]} scale={1.2} /> 
                    <ChristmasTree position={[3, 0, -3]} scale={1.0} />

                    {/* 🎅 [NEW] 움직이는 산타할아버지 추가 */}
                    <SantaClaus radius={2.1} speed={0.5} />

                    {/* ☃️ 눈사람 추가 (위치 조정됨) */}
                    <Snowman position={[4, 0, -3]} scale={0.7} rotation={[0, -0.5, 0]} />

                    {/* 🏠 산타 마을 집들 (배경) */}
                    <SantaHouse position={[-4, 0, -5]} rotation={[0, 0.3, 0]} scale={1.5} />
                    <SantaHouse position={[2, 0, -6]} rotation={[0, -0.2, 0]} scale={1.2} />
                    <SantaHouse position={[5, 0, -4]} rotation={[0, -0.5, 0]} scale={1.0} />

                    {/* 🍼 [NEW] 시럽통 배치 (로봇 옆) */}
                    {/* 딸기 시럽 (핑크) */}
                    <SyrupBottle 
                        position={[2.5, 0, 2]} 
                        color="#D81B60" 
                        isActive={selectedSyrup === 'strawberry_syrup'} 
                        label="Strawberry" 
                    />
                    {/* 초코 시럽 (갈색) */}
                    <SyrupBottle 
                        position={[3.2, 0, 2]} 
                        color="#5D4037" 
                        isActive={selectedSyrup === 'choco_syrup'} 
                        label="Choco" 
                    />
                </Stage>
              </Suspense>
              <OrbitControls makeDefault minPolarAngle={0} maxPolarAngle={Math.PI / 2.1} />
            </Canvas>
          </div>

          <div className="h-20 grid grid-cols-12 gap-4">
            {/* 기존 버튼 위치 (약 1240번 라인 부근) */}
            <button
              onClick={handlePreOrderCheck}
              disabled={isRunning || !isToppingComplete} // 8개가 아니면 클릭 불가
              className={`col-span-7 rounded-2xl text-xl font-bold text-white transition flex items-center justify-center 
    ${isRunning || !isToppingComplete
                  ? 'bg-gray-400 cursor-not-allowed' // 8개가 아니면 회색
                  : 'bg-blue-600 shadow-lg hover:bg-blue-500' // 8개면 파란색
                }`}
            >
              {isToppingComplete ? "📄 주문서 전송" : `토핑 8개를 채워주세요 (${currentTotalToppings}/8)`}
            </button>             {/* <button onClick={handleStop} disabled={!isRunning} className="col-span-2 rounded-2xl font-bold text-white bg-red-500 shadow-lg hover:bg-red-400 transition">⛔ 정지</button> */}
             <button onClick={() => setShowStatusModal(true)} className="col-span-3 rounded-2xl font-bold text-slate-700 bg-white border-2 border-slate-200 shadow-lg hover:bg-slate-50 transition flex items-center justify-center gap-2">📊 상태 보기</button>
          </div>
        </section>
      </main>

                  
      <VoiceCounselor 
        onLog={addLog}
        currentContext={aiContext} // 현재까지 채워진 정보 전달
        onUpdateContext={(newFields) => setAiContext(prev => ({ ...prev, ...newFields }))} // 정보 업데이트용
        onOrderConfirm={(data) => { 
          setAutoOrderData(data); 
          setShowOrderCheckModal(true); 
        }} 
      />

      <Popup 
        isOpen={showPickupModal}
        onClose={() => setShowPickupModal(false)}
        onNext={handlePickupNext} // 확인 누르면 handleStart 실행
        orderSummary={{
            size: cakeSize,
            toppingCount: currentTotalToppings
        }}
      />
      <PaymentModal
        isOpen={showPaymentModal}
        onClose={() => setShowPaymentModal(false)}
        onPaymentComplete={handlePaymentComplete}
        pickupData={tempPickupData}
        price={cakeSize === "1호" ? 25000 : 35000} // 사이즈별 가격 예시
      />
      <OrderSuccessModal
        isOpen={showOrderSuccessModal}
        onClose={() => setShowOrderSuccessModal(false)}
        orderId={confirmedOrderInfo.id}
        waitTime={confirmedOrderInfo.waitTime}
      />
      
      {/* 상태 모달 */}
      {showStatusModal && <RobotStatusModal onClose={() => setShowStatusModal(false)} />}
    </div>
  );
}