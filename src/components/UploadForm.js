import React from 'react';
import { useState } from 'react';

const UploadForm = ({ setEntityLink, setEntityID, setIsProcessing }) => {

    const [uploadFile, setUploadFile] = useState(undefined);
    const [isUploading, setIsUploading] = useState(false);

    //pass state to component, move state to home - modify state of parent componenet

    function submitUploadRequest(event) {
        event.preventDefault();
        setIsUploading(true);
        const postUrl = `https://api.echo3D.com/upload`;
        const formData = new FormData();
        formData.append('key', process.env.REACT_APP_ECHO_3D_API_KEY);
        formData.append('email', process.env.REACT_APP_ECHO_3D_USER_EMAIL);
        formData.append('target_type', 2);
        formData.append('hologram_type', 2);
        formData.append('type', 'upload');
        formData.append('file_model', uploadFile);
        formData.append('secKey', process.env.REACT_APP_ECHO_3D_SECURITY_KEY);
        fetch(postUrl, { method: 'POST', header: { 'Content-Type': 'multipart/form-data' }, body: formData })
            .then((response) => response.json())
            .then(response => {
                setEntityLink(response['additionalData']['shortURL']);
                setEntityID(response['id']);
            });
        setIsProcessing(true);
    }

    return (
        <div className="upload-form-container">
            <h3>UPLOAD</h3>
            <h6 id="upload-instr">Upload a .glb files only</h6>

            {isUploading ? (
                <div className="spinner"></div>
            ) : (
                <form encType="multipart/form-data" id="upload-form" onSubmit={submitUploadRequest}>
                    <input className="file-chooser btn btn-outline-dark mb-3 mx-3" type="file" name="file_model" accept=".glb" onChange={(event) => setUploadFile(event.target.files[0])} />
                    <input className="btn btn-dark mb-3 mx-3" type="submit" />
                </form>
            )}

        </div>
    );


}

export default UploadForm;