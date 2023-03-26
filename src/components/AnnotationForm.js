import React from 'react';
import { useState } from 'react';

const AnnotationForm = () => {
  const [currentAnnotation, setCurrentAnnotation] = useState(1);
  const [annotationContent, setAnnotationContent] = useState('');
  const [annotationPosition, setAnnotationPosition] = useState('');

  function submitAnnotationRequest(data, value) {
    const postUrl = `https://api.echo3D.com/post?key=${process.env.REACT_APP_ECHO_3D_API_KEY}&entry=${process.env.REACT_APP_ENTITY_ID}&data=${data}&value=${value}&secKey=${process.env.REACT_APP_ECHO_3D_SECURITY_KEY}`;

    fetch(postUrl, { method: 'POST' });
  }

  function addAnnotation(event) {
    event.preventDefault();

    submitAnnotationRequest(`annotation${currentAnnotation}_content`, annotationContent);
    submitAnnotationRequest(`annotation${currentAnnotation}_position`, annotationPosition);
    submitAnnotationRequest(`annotation${currentAnnotation}_normal`, "0,0,0");

    console.log(`Created annotation ${currentAnnotation} with text ${annotationContent} at ${annotationPosition}`);

    setCurrentAnnotation(currentAnnotation + 1);
  }

  return (
    <div className="annotation-forms">
      <form id="annotation-inputs">
        <input type="text" placeholder="Content" value={annotationContent} onChange={(event) => setAnnotationContent(event.target.value)}/>
        <input type="text" placeholder="Position" value={annotationPosition} onChange={(event) => setAnnotationPosition(event.target.value)}/>
        <input type="button" value="Submit" onClick={addAnnotation} />
      </form>
    </div>
  );
}

export default AnnotationForm;