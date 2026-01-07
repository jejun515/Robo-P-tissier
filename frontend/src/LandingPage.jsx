import React, { Suspense, useRef, useState } from "react";
import { Canvas, useFrame } from "@react-three/fiber";
import { Float, Stars, Sparkles, ContactShadows, Environment } from "@react-three/drei";
import * as THREE from "three";

/**
 * 🎂 3D 케이크 컴포넌트 (회전하는 메인 오브젝트)
 * 3단 케이크 디자인으로 제작
 */
function FloatingCake() {
  const groupRef = useRef();
  const [hovered, setHovered] = useState(false);
  
  useFrame((state, delta) => {
    if (groupRef.current) {
      groupRef.current.rotation.y += delta * 0.3;
      groupRef.current.position.y = Math.sin(state.clock.elapsedTime * 0.5) * 0.1;
    }
  });

  return (
    <Float speed={1.5} rotationIntensity={0.5} floatIntensity={0.8}>
      <group 
        ref={groupRef} 
        scale={[1.2, 1.2, 1.2]}
        onPointerOver={() => setHovered(true)}
        onPointerOut={() => setHovered(false)}
      >
        {/* 하단 케이크 층 */}
        <mesh position={[0, -0.8, 0]} castShadow>
          <cylinderGeometry args={[1.5, 1.5, 0.6, 32]} />
          <meshStandardMaterial 
            color="#FFE5EC" 
            roughness={0.3} 
            metalness={0.1}
            emissive="#FFB3D9"
            emissiveIntensity={hovered ? 0.3 : 0.1}
          />
        </mesh>

        {/* 중단 케이크 층 */}
        <mesh position={[0, -0.2, 0]} castShadow>
          <cylinderGeometry args={[1.2, 1.2, 0.6, 32]} />
          <meshStandardMaterial 
            color="#FFF0F5" 
            roughness={0.3} 
            metalness={0.1}
            emissive="#FFC0E3"
            emissiveIntensity={hovered ? 0.3 : 0.1}
          />
        </mesh>

        {/* 상단 케이크 층 */}
        <mesh position={[0, 0.4, 0]} castShadow>
          <cylinderGeometry args={[0.9, 0.9, 0.6, 32]} />
          <meshStandardMaterial 
            color="#FFFFFF" 
            roughness={0.2} 
            metalness={0.2}
            emissive="#FFD6E8"
            emissiveIntensity={hovered ? 0.4 : 0.15}
          />
        </mesh>

        {/* 케이크 토핑 (체리) */}
        <mesh position={[0, 1, 0]} castShadow>
          <sphereGeometry args={[0.15, 16, 16]} />
          <meshStandardMaterial 
            color="#FF2E63" 
            roughness={0.1} 
            metalness={0.5}
            emissive="#FF2E63"
            emissiveIntensity={0.5}
          />
        </mesh>

        {/* 케이크 데코레이션 - 크림 라인들 */}
        {[0, 1, 2, 3, 4, 5].map((i) => (
          <mesh key={i} position={[
            Math.cos((i / 6) * Math.PI * 2) * 1.5,
            -0.5,
            Math.sin((i / 6) * Math.PI * 2) * 1.5
          ]}>
            <sphereGeometry args={[0.08, 8, 8]} />
            <meshStandardMaterial color="#FFEEF8" roughness={0.2} metalness={0.3} />
          </mesh>
        ))}

        {/* 반짝이는 입자들 (케이크 주변) */}
        <Sparkles count={80} scale={3.5} size={3} speed={0.3} opacity={0.8} color="#FFD9E8" />
      </group>
    </Float>
  );
}

/**
 * ✨ 배경 장식용 3D 요소들
 */
function BackgroundElements() {
  return (
    <>
      <Stars radius={150} depth={60} count={5000} factor={5} saturation={0} fade speed={0.5} />
      
      {/* 크리스탈 조각들 */}
      <Float speed={2} rotationIntensity={3} floatIntensity={2} position={[-5, 3, -6]}>
        <mesh>
          <octahedronGeometry args={[0.8, 0]} />
          <meshStandardMaterial 
            color="#E0BBE4" 
            transparent 
            opacity={0.6}
            roughness={0.1}
            metalness={0.9}
            emissive="#E0BBE4"
            emissiveIntensity={0.2}
          />
        </mesh>
      </Float>

      <Float speed={3} rotationIntensity={2} floatIntensity={3} position={[5, -2, -4]}>
        <mesh>
          <dodecahedronGeometry args={[0.6, 0]} />
          <meshStandardMaterial 
            color="#FFDFD3" 
            transparent 
            opacity={0.5}
            roughness={0.2}
            metalness={0.8}
            emissive="#FFDFD3"
            emissiveIntensity={0.3}
          />
        </mesh>
      </Float>

      <Float speed={2.5} rotationIntensity={1} floatIntensity={2} position={[4, 3, -5]}>
        <mesh>
          <torusGeometry args={[0.5, 0.15, 16, 32]} />
          <meshStandardMaterial 
            color="#FEC8D8" 
            roughness={0.1}
            metalness={0.9}
            emissive="#FEC8D8"
            emissiveIntensity={0.2}
          />
        </mesh>
      </Float>

      {/* 부드러운 파티클 효과 */}
      <Sparkles count={150} scale={15} size={1.5} speed={0.2} opacity={0.3} color="#FFE5EC" />
    </>
  );
}

/**
 * 🚀 [메인] 랜딩 페이지 컴포넌트
 * 수정사항: 관리자 버튼을 텍스트가 포함된 명확한 버튼으로 변경했습니다.
 */
export default function LandingPage({ onStart = () => alert("메인 앱으로 이동합니다!"), onAdmin }) {
  const [isHovered, setIsHovered] = useState(false);

  return (
    <div className="relative w-full h-screen overflow-hidden font-sans selection:bg-pink-400 selection:text-white">
      
      {/* ⚙️ 관리자 설정 버튼 (텍스트형 버튼으로 변경) */}
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

      {/* =================================================================
          Layer 1: 3D Canvas Background
          ================================================================= */}
      <div className="absolute inset-0 z-0 bg-gradient-to-br from-rose-100 via-pink-50 to-purple-100">
        {/* 추가 그라디언트 오버레이 */}
        <div className="absolute inset-0 bg-gradient-to-t from-purple-900/20 via-transparent to-pink-900/10" />
        
        <Canvas camera={{ position: [0, 0, 9], fov: 50 }} shadows>
          <ambientLight intensity={0.6} />
          <spotLight 
            position={[10, 15, 10]} 
            angle={0.2} 
            penumbra={1} 
            intensity={2} 
            castShadow 
            color="#FFE5EC"
          />
          <pointLight position={[-10, -10, -10]} intensity={0.8} color="#E0BBE4" />
          <pointLight position={[10, 5, 5]} intensity={0.6} color="#FEC8D8" />
          
          <Suspense fallback={null}>
            <Environment preset="sunset" />
            <FloatingCake />
            <BackgroundElements />
            <ContactShadows 
              position={[0, -4, 0]} 
              opacity={0.25} 
              scale={25} 
              blur={2.5} 
              far={5} 
              color="#8B5CF6"
            />
          </Suspense>
        </Canvas>
      </div>

      {/* 빛 효과 오버레이 */}
      <div className="absolute inset-0 z-[5] pointer-events-none">
        <div className="absolute top-0 left-1/4 w-96 h-96 bg-pink-300/20 rounded-full blur-3xl animate-pulse" />
        <div className="absolute bottom-0 right-1/4 w-96 h-96 bg-purple-300/20 rounded-full blur-3xl animate-pulse" style={{ animationDelay: '1s' }} />
      </div>

      {/* =================================================================
          Layer 2: UI Overlay
          ================================================================= */}
      <div className="relative z-10 w-full h-full flex flex-col justify-center items-center px-6">
        
        {/* 상단 네비게이션 */}
        <nav className="absolute top-0 left-0 w-full p-6 md:p-8 flex justify-between items-center pointer-events-none">
            <div className="flex items-center gap-3 pointer-events-auto">
                <div className="relative">
                    <div className="w-3 h-3 bg-gradient-to-r from-pink-500 to-rose-500 rounded-full animate-pulse" />
                    <div className="absolute inset-0 w-3 h-3 bg-gradient-to-r from-pink-500 to-rose-500 rounded-full animate-ping opacity-75" />
                </div>
                <span className="text-gray-800 font-black text-xl tracking-wider">
                    ROKEY
                </span>
            </div>
            {/* 우측 상단은 이제 Admin 버튼이 차지하므로 메뉴는 숨기거나 위치 조정 */}
            <div className="hidden md:flex gap-8 text-sm text-gray-700 font-semibold pointer-events-auto pr-16">
                <button className="hover:text-pink-600 transition-colors duration-200 hover:scale-105 transform">About</button>
                <button className="hover:text-pink-600 transition-colors duration-200 hover:scale-105 transform">Gallery</button>
                <button className="hover:text-pink-600 transition-colors duration-200 hover:scale-105 transform">Contact</button>
            </div>
        </nav>

        {/* 메인 텍스트 컨텐츠 */}
        <main className="text-center flex flex-col items-center max-w-5xl mx-auto mt-8">
            
            {/* 브랜드 뱃지 */}
            <div className="animate-in fade-in slide-in-from-top-4 duration-1000 mb-8">
                 <span className="inline-flex items-center gap-2 px-5 py-2 rounded-full bg-gradient-to-r from-pink-500/10 to-purple-500/10 border border-pink-300/30 backdrop-blur-xl shadow-lg text-pink-700 text-xs md:text-sm font-bold tracking-widest uppercase">
                    <span className="animate-pulse">✨</span> AI-Powered Baking Excellence
                 </span>
            </div>

            {/* 헤드라인 */}
            <h1 className="animate-in zoom-in duration-1000 delay-100 mb-8 leading-none">
              <div className="text-7xl md:text-8xl lg:text-9xl font-black tracking-tighter">
                <span className="block text-transparent bg-clip-text bg-gradient-to-r from-pink-600 via-rose-500 to-purple-600 drop-shadow-sm">
                  DREAM
                </span>
              </div>
              <div className="text-6xl md:text-7xl lg:text-8xl font-black tracking-tight mt-2 text-gray-800">
                CAKE LAB
              </div>
            </h1>

            {/* 서브 카피 */}
            <p className="animate-in fade-in slide-in-from-bottom-4 duration-1000 delay-300 text-xl md:text-2xl text-gray-700 font-light max-w-3xl mx-auto mb-12 leading-relaxed">
               로봇 셰프가 당신의 꿈을 현실로 구현합니다.<br className="hidden md:block"/>
               <span className="inline-block mt-2 px-4 py-1 bg-gradient-to-r from-pink-500/20 to-purple-500/20 rounded-full">
                 <strong className="text-pink-700 font-bold">세상에 단 하나뿐인</strong> 나만의 케이크를 만나보세요.
               </span>
            </p>

            {/* CTA 버튼 */}
            <div className="animate-in fade-in slide-in-from-bottom-8 duration-1000 delay-500">
                <button
                    onClick={onStart}
                    onMouseEnter={() => setIsHovered(true)}
                    onMouseLeave={() => setIsHovered(false)}
                    className="group relative inline-flex items-center justify-center px-12 py-6 bg-gradient-to-r from-pink-500 via-rose-500 to-purple-600 text-white rounded-full overflow-hidden transition-all duration-500 hover:scale-110 hover:shadow-[0_20px_60px_rgba(236,72,153,0.4)] focus:outline-none focus:ring-4 focus:ring-pink-500/50 active:scale-95"
                >
                    {/* 버튼 내부 글로우 효과 */}
                    <div className="absolute inset-0 bg-gradient-to-r from-white/0 via-white/20 to-white/0 transform -skew-x-12 group-hover:translate-x-full transition-transform duration-1000" />
                    
                    {/* 버튼 텍스트 */}
                    <span className="relative z-10 text-xl md:text-2xl font-black tracking-wide mr-3 drop-shadow-lg">
                        케이크 주문하기
                    </span>
                    <svg 
                        className={`w-7 h-7 relative z-10 transition-all duration-300 ${isHovered ? 'translate-x-2 scale-110' : ''}`} 
                        fill="none" stroke="currentColor" viewBox="0 0 24 24" strokeWidth="3"
                    >
                        <path strokeLinecap="round" strokeLinejoin="round" d="M17 8l4 4m0 0l-4 4m4-4H3" />
                    </svg>
                    
                    {/* 버튼 하단 반짝임 */}
                    <div className="absolute -bottom-2 left-1/2 -translate-x-1/2 w-3/4 h-1 bg-gradient-to-r from-transparent via-white/50 to-transparent blur-sm group-hover:via-white/80 transition-all" />
                </button>
                
                <div className="mt-6 flex items-center justify-center gap-2 text-xs text-gray-600 font-medium">
                    <div className="w-2 h-2 bg-green-500 rounded-full animate-pulse" />
                    <span>AI 디자인 엔진 가동 중</span>
                </div>
            </div>

            {/* 신뢰 배지들 */}
            <div className="animate-in fade-in duration-1000 delay-700 mt-16 flex flex-wrap justify-center gap-8 opacity-70">
                <div className="flex items-center gap-2 text-gray-600 text-sm">
                    <svg className="w-5 h-5 text-pink-500" fill="currentColor" viewBox="0 0 20 20">
                        <path d="M9.049 2.927c.3-.921 1.603-.921 1.902 0l1.07 3.292a1 1 0 00.95.69h3.462c.969 0 1.371 1.24.588 1.81l-2.8 2.034a1 1 0 00-.364 1.118l1.07 3.292c.3.921-.755 1.688-1.54 1.118l-2.8-2.034a1 1 0 00-1.175 0l-2.8 2.034c-.784.57-1.838-.197-1.539-1.118l1.07-3.292a1 1 0 00-.364-1.118L2.98 8.72c-.783-.57-.38-1.81.588-1.81h3.461a1 1 0 00.951-.69l1.07-3.292z" />
                    </svg>
                    <span className="font-semibold">4.9/5.0</span>
                    <span>고객 만족도</span>
                </div>
                <div className="flex items-center gap-2 text-gray-600 text-sm">
                    <svg className="w-5 h-5 text-purple-500" fill="currentColor" viewBox="0 0 20 20">
                        <path fillRule="evenodd" d="M6.267 3.455a3.066 3.066 0 001.745-.723 3.066 3.066 0 013.976 0 3.066 3.066 0 001.745.723 3.066 3.066 0 012.812 2.812c.051.643.304 1.254.723 1.745a3.066 3.066 0 010 3.976 3.066 3.066 0 00-.723 1.745 3.066 3.066 0 01-3.976 0 3.066 3.066 0 00-1.745-.723 3.066 3.066 0 01-2.812-2.812 3.066 3.066 0 00-.723-1.745 3.066 3.066 0 010-3.976 3.066 3.066 0 00.723-1.745 3.066 3.066 0 012.812-2.812zm7.44 5.252a1 1 0 00-1.414-1.414L9 10.586 7.707 9.293a1 1 0 00-1.414 1.414l2 2a1 1 0 001.414 0l4-4z" clipRule="evenodd" />
                    </svg>
                    <span className="font-semibold">ISO 인증</span>
                </div>
                <div className="flex items-center gap-2 text-gray-600 text-sm">
                    <svg className="w-5 h-5 text-rose-500" fill="currentColor" viewBox="0 0 20 20">
                        <path fillRule="evenodd" d="M3.172 5.172a4 4 0 015.656 0L10 6.343l1.172-1.171a4 4 0 115.656 5.656L10 17.657l-6.828-6.829a4 4 0 010-5.656z" clipRule="evenodd" />
                    </svg>
                    <span className="font-semibold">10,000+</span>
                    <span>완성된 케이크</span>
                </div>
            </div>
        </main>

        {/* 하단 정보 */}
        <footer className="absolute bottom-8 w-full text-center px-4">
            <div className="flex flex-col md:flex-row justify-center items-center gap-3 text-xs text-gray-600 tracking-wider font-medium">
                <span className="flex items-center gap-2">
                    <span className="text-pink-500">●</span> Powered by Doosan Robotics
                </span>
                <span className="hidden md:block text-gray-400">|</span>
                <span>Advanced AI Design Engine v3.0</span>
                <span className="hidden md:block text-gray-400">|</span>
                <span>Made with ❤️ in Korea</span>
            </div>
        </footer>

      </div>
    </div>
  );
}