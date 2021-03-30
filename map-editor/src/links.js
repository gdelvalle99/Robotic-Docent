const baseLink = "http://localhost:5000"
const mapLink = baseLink+"/floor/map/get"
const tourLink = baseLink+"/tour/start"
const floor_id = "476411ee-df58-4b98-8da6-4514d2fc0433"
const floorLink = baseLink+"/floor/exhibits?id="+floor_id
const newExhibitLink = baseLink+"/exhibit/new"
const existingExhibitLink = baseLink+"/exhibit/update"
const deleteExhibitLink = baseLink+"/exhibit/delete?id="
const demoLink = "http://decd071e058d.ngrok.io/start_tour"


const validateLink = baseLink + "/auth/validate"


export {baseLink, mapLink, tourLink, floorLink, floor_id, newExhibitLink, existingExhibitLink, deleteExhibitLink, demoLink, validateLink}
