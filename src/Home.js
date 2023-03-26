import React from 'react';
import AnnotationForm from './components/AnnotationForm';
import "./styles/Home.css";
import UploadForm from './components/UploadForm';
import { useState } from 'react';

function Home() {
  const [entityLink, setEntityLink] = useState(undefined);
  const [entityID, setEntityID] = useState(undefined);

  return (
    <div className="homepage-container">
      {entityID === undefined &&
        <div className="upload-view">
          <UploadForm setEntityLink={setEntityLink} setEntityID={setEntityID} />
        </div>
      }
      {entityID !== undefined && entityLink !== undefined &&
        <div className="model-display">
          <h1>GUIDEROOM Test File</h1>
          <iframe id="model-view" src={entityLink} title="echo3D WebAR iframe element" />
        </div>
      }
    </div>
  );
}

export default Home;
