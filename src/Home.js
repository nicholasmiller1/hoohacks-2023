import React from 'react';
import "./styles/Home.css";
import UploadForm from './components/UploadForm';
import { useState } from 'react';

function Home() {
  const [entityLink, setEntityLink] = useState(undefined);
  const [entityID, setEntityID] = useState(undefined);
  const [isProcessing, setIsProcessing] = useState(false);
  const [processingResults, setProcessingResults] = useState(undefined);

  return (
    <div className="homepage-container mx-auto">
      {entityID === undefined &&
        <div className="upload-view">
          <h2>WELCOME, </h2>
          <h5>Guideroom aims to close the gap between architecture and accessibility. We utilize lidar data and detection algorithms to rate how accessible a building, room, or design choice is.</h5>
          <br />
          <div className="upload-container">
            <h3>GET STARTED</h3>
            {/* Arrow taken from Iconify: https://icon-sets.iconify.design/material-symbols/keyboard-arrow-down-rounded/ */}
            <svg xmlns="http://www.w3.org/2000/svg" width="60" height="60" viewBox="0 0 24 24"><path fill="currentColor" d="M12 14.975q-.2 0-.388-.075t-.312-.2l-4.6-4.6q-.275-.275-.275-.7t.275-.7q.275-.275.7-.275t.7.275l3.9 3.9l3.9-3.9q.275-.275.7-.275t.7.275q.275.275.275.7t-.275.7l-4.6 4.6q-.15.15-.325.213t-.375.062Z" /></svg>
            <UploadForm setEntityLink={setEntityLink} setEntityID={setEntityID} setIsProcessing={setIsProcessing}/>
          </div>
        </div>
      }
      {entityID !== undefined && entityLink !== undefined &&
        <div className="model-display">
          {isProcessing ? (
            <div className='process-loader'>
              <h3>Your file is being processed...</h3>
              <div className="spinner"></div>
            </div>
          ) : (
            <h3>Your File Results</h3>
          )}
          <iframe id="model-view" src={entityLink} title="echo3D WebAR iframe element" />
          <p>{processingResults}</p>
        </div>
      }
    </div>
  );
}

export default Home;
