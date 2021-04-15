const baseLink = "http://localhost:5000"
const mapLink = baseLink+"/floor/map/get"
const tourLink = baseLink+"/tour/start"
const floor_id = "508ac281-29cd-4ec8-b9b9-84046f88aa94"
const floorLink = baseLink+"/floor/exhibits?id="+floor_id
const newExhibitLink = baseLink+"/exhibit/new"
const existingExhibitLink = baseLink+"/exhibit/update"
const deleteExhibitLink = baseLink+"/exhibit/delete?id="
const exhibitLink = baseLink+"/exhibit/pieces?id="
const newPieceLink = baseLink+"/piece/new"
const demoLink = "http://decd071e058d.ngrok.io/start_tour"


const validateLink = baseLink + "/auth/validate"


export {baseLink, mapLink, tourLink, floorLink, floor_id, newExhibitLink, existingExhibitLink, deleteExhibitLink, exhibitLink, newPieceLink, demoLink, validateLink}
