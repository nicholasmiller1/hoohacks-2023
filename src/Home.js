import React from 'react';
import "./styles/Home.css";

function Home() {
  return (
    <div className="homepage-container">
      <div className="container w-100 justify-content-center"></div>
      <div className="model-display">
        <h1>GUIDEROOM Test File</h1>
        <iframe
          id="model-view"
          src="https://go.echo3d.co/QsiU"
          title="echo3D WebAR iframe element">
        </iframe>
      </div>
    </div>
  );
}

export default Home;
